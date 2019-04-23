import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from nav_msgs.msg import *
import math
import tf
import time
import heapq
import numpy
from std_msgs.msg import Float32MultiArray


class Motor_Controller:
	def __init__(self):
		self.BURGER_MAX_LIN_VEL = 0.22
		self.BURGER_MAX_ANG_VEL = 2.00

		self.vel_low_limit = 0.05

		self.WAFFLE_MAX_LIN_VEL = 0.26
		self.WAFFLE_MAX_ANG_VEL = 1.82

		self.LIN_VEL_STEP_SIZE = 0.01
		self.ANG_VEL_STEP_SIZE = 0.1

		self.turtlebot3_model = ""

		self.coordinates = {'a':{'x':0,'y':0}, 'b':{'x':0,'y':1}, 'c':{'x':0,'y':2}, 'd':{'x':1,'y':2}, 'e':{'x':1,'y':1}, 'f':{'x':2,'y':2}, 'g':{'x':2,'y':4}, 'h':{'x':0,'y':3}, 'i':{'x':1,'y':4}}
		self.goal = []

		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_yaw = 0.0

		self.target_linear_vel   = 0.0
		self.target_angular_vel  = 0.0 
		self.control_linear_vel  = 0.0
		self.control_angular_vel = 0.0
		self.goal_xcoordinate = 0.0
		self.goal_ycoordinate = 0.0

		self.theta_kp = 1.0/3.0
		self.theta_kd = 0.0
		self.theta_ki = 0.0

		self.vel_kp = 1.0/6.0
		self.vel_kd = 0.0
		self.vel_ki = 0.0
		
		self.prev_dist = 0.0
		self.prev_theta = 0.0
		self.int_dist = 0.0
		self.int_theta = 0.0

	def init_pub_sub(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_data_callback)
		self.sub = rospy.Subscriber('final_path', Float32MultiArray, self.get_goal)

	def vels(self,target_linear_vel, target_angular_vel):
		return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

	def makeSimpleProfile(self,output, input, slop):
		if input > output:
			output = min( input, output + slop )
		elif input < output:
			output = max( input, output - slop )
		else:
			output = input

		return output

	def odom_data_callback(self,data):
		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		robot_orientation = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
		self.robot_yaw = robot_orientation[2]
		#print self.robot_yaw

	def constrain(self,input, low, high):
		if input < low:
			input = low
		elif input > high:
			input = high
		else:
			input = input

		return input

	def checkLinearLimitVelocity(self,vel):
		if self.turtlebot3_model == "burger":
			vel = self.constrain(vel, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)
		elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
			vel = self.constrain(vel, -self.WAFFLE_MAX_LIN_VEL, self.WAFFLE_MAX_LIN_VEL)
		else:
			vel = self.constrain(vel, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)

		return vel

	def checkAngularLimitVelocity(self,vel):
		if self.turtlebot3_model == "burger":
			vel = self.constrain(vel, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)
		elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
			vel = self.constrain(vel, -self.WAFFLE_MAX_ANG_VEL, self.WAFFLE_MAX_ANG_VEL)
		else:
			vel = self.constrain(vel, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)

		return vel

	def get_goal(self,msg):
		self.goal = msg.data
		self.goal_xcoordinate = self.goal[0]
		self.goal_ycoordinate = self.goal[1]


	def calculate_velocity(self):
		velx = 0.0
		angle = 0.0
		dist = math.sqrt((self.goal_xcoordinate - self.robot_x)**2 + (self.goal_ycoordinate - self.robot_y)**2)

		if ( (self.goal_xcoordinate-self.robot_x)!=0.0 ):
			if((self.goal_ycoordinate-self.robot_y)>=0.0 and (self.goal_xcoordinate - self.robot_x)>0.0):
				angle = abs(math.atan((self.goal_ycoordinate - self.robot_y)/(self.goal_xcoordinate - self.robot_x))) - self.robot_yaw
			elif((self.goal_ycoordinate-self.robot_y)<=0.0 and (self.goal_xcoordinate - self.robot_x)>0.0):
				angle = -abs(math.atan((self.goal_ycoordinate - self.robot_y)/(self.goal_xcoordinate - self.robot_x))) - self.robot_yaw
			elif((self.goal_ycoordinate-self.robot_y)>=0.0 and (self.goal_xcoordinate - self.robot_x)<0.0):
				if(self.robot_yaw<-0.1):
					self.robot_yaw += 2*math.pi
				angle = math.pi - abs(math.atan((self.goal_ycoordinate - self.robot_y)/(self.goal_xcoordinate - self.robot_x))) - self.robot_yaw
			elif((self.goal_ycoordinate-self.robot_y)<=0.0 and (self.goal_xcoordinate - self.robot_x)<0.0):
				if(self.robot_yaw<-0.1):
					self.robot_yaw += 2*math.pi
				angle = math.pi + abs(math.atan((self.goal_ycoordinate - self.robot_y)/(self.goal_xcoordinate - self.robot_x))) - self.robot_yaw
		else:	
			if((self.goal_ycoordinate-self.robot_y)>0.0):
				angle = (math.pi)/2.0- self.robot_yaw
			elif((self.goal_ycoordinate-self.robot_y)<0.0):
				angle = -(math.pi)/2.0 - self.robot_yaw
			else:
				angle = 0.0

		omega = self.theta_kp*angle + self.theta_kd*(angle-self.prev_theta) + self.theta_ki*(self.int_theta)
		self.prev_theta = angle
		self.int_theta += angle

		if(angle<0.1 and angle>-0.1 and dist > 0.05):
			if(dist>0.05):
				velx = self.vel_kp*dist + self.vel_kd*(dist - self.prev_dist) + self.vel_ki*(self.int_dist)
				if (velx<self.vel_low_limit):
					velx = self.vel_low_limit
				self.int_dist+=dist - self.prev_dist

		velx = self.checkLinearLimitVelocity(velx)
		omega = self.checkAngularLimitVelocity(omega)
		#print velx,"    ",omega,"   ", self.robot_yaw
		return velx,omega

	def main(self):
		rospy.init_node('motor_controller')
		self.init_pub_sub()
		self.turtlebot3_model = rospy.get_param("model", "burger")

		self.target_linear_vel   = 0.0
		self.target_angular_vel  = 0.0 
		self.control_linear_vel  = 0.0
		self.control_angular_vel = 0.0

		try:
			while not rospy.is_shutdown():
				print self.goal
				twist = Twist()
				if (len(self.goal)!=0):
					target_linear_vel,target_angular_vel = self.calculate_velocity()
					twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

					twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

					self.pub.publish(twist)

		finally:
			twist = Twist()
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
			self.pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__=="__main__":
	controller = Motor_Controller()
	controller.main()