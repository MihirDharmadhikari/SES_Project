#! /usr/bin/env python



import rospy
import math
import numpy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon,Point32,PolygonStamped 
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import PolygonArray
from ses.msg import Ipoly,Exp_msg,Cordi#for polygon conversion
from nav_msgs.msg import Odometry

pi = math.pi

bot_x = 0.
bot_y = 0.
bot_yaw = 0.


#for decreasing the coordinate val 
def decrease_in_direction(val): 
	if val!="inf":
		val-=0.15 #change this vaue according to bot size
	return val

#for increasing the coordinate val
def increase_in_direction(val):
	if val!="inf":
		val+=0.3 #-4 because the value is changed in decrease_in_direction
	return val


#for converting the point array to a polygon using jsk_recognition_msgs 
def getting_cordi(A,B,shu):
	theta_increment=shu
	re=PolygonArray()
	re.header.frame_id="base_scan"
	roar=PolygonStamped()
	roar.header.frame_id="base_scan"
	pt=Exp_msg()
	count=0
	baby=Ipoly()
	for i in range(len(A)):
		if str(A[i])!="inf":
			if count==0:
				for t in range(3,1,-1):
					theta1=(i-t)*theta_increment
					x=A[i]*math.cos(theta1)
					y=A[i]*math.sin(theta1)       
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
			count+=1
			theta1=(i)*theta_increment
			x=A[i]*math.cos(theta1)
			y=A[i]*math.sin(theta1)       
			roar.polygon.points.append(Point32(x,y,0))
			pt.bliss.append(Cordi(x,y,0))
			if i==len(A)-1 or str(A[i+1])=="inf" or abs(A[i+1]-A[i])>0.5:
				for t in range(1,5):
					theta1=(t+i)*theta_increment
					x=A[i]*math.cos(theta1)
					y=A[i]*math.sin(theta1)       
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				for t in range(3,0,-1):
					theta1=(t+i)*theta_increment
					x=B[i]*math.cos(theta1)
					y=B[i]*math.sin(theta1)       
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				for j in range(i,i-count,-1):
					theta2=(j)*theta_increment
					x=B[j]*math.cos(theta2)
					y=B[j]*math.sin(theta2)       
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				for t in range(-1,-3,-1):
					theta2=(j+t)*theta_increment
					x=B[j]*math.cos(theta2)
					y=B[j]*math.sin(theta2)       
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))			
				baby.eternal_bliss.append(pt)
				pt=Exp_msg()
				count=0
				re.polygons.append(roar)
				roar=PolygonStamped()
				roar.header.frame_id="base_scan"
	pubf=rospy.Publisher("ol2",Ipoly,queue_size=0)
	pubf.publish(baby)

	# print "bot yaw calculator ", bot_yaw
	

	final_polys = PolygonArray()
	# print "New"
	for p in re.polygons:
		# print " "
		temp_poly = PolygonStamped()
		temp_poly.header.frame_id = "odom"
		for point in p.polygon.points:
			temp_point = numpy.array([[point.x],[point.y]])
			# print temp_point
			# print point.x, point.y
			# print "bot yaw calculator ", bot_yaw
			cos=math.cos(-bot_yaw)
			sin=math.sin(-bot_yaw)
			# print cos, sin
			rot_mat=numpy.array([[cos , sin],[-sin , cos]])
			# rot_mat=numpy.array([[1 , 0],[0 , 1]])
			trans_mat=numpy.array([[bot_x], [bot_y]])

			rot_point = numpy.matmul(rot_mat, temp_point)
			# print rot_point
			trans_point = rot_point + trans_mat
			# print trans_point[0], trans_point[1]
			# point.x = trans_point[0]
			# point.y = trans_point[1]

			t_point = Point32()
			t_point.x = trans_point[0]
			t_point.y = trans_point[1]
			temp_poly.polygon.points.append(t_point)

		final_polys.polygons.append(temp_poly)
	final_polys.header.frame_id = "odom"
	return final_polys



#adding additional points at the end of the obstacle
def increasing_array(input_data):
	
	count=len(input_data)
	i=1
	while i<count:
		if (str(input_data[i])=='inf' and str(input_data[i-1])!='inf'):
			insert_element=input_data[i-1]
			for j in range(10):
				if i+j==360:
					break
				input_data[i+j]=insert_element
			i+=11
		elif (str(input_data[i-1])=='inf' and str(input_data[i])!='inf'):
			insert_element=input_data[i]
			for j in range(1,11):
				if i-j==0:
					break
				else:
					input_data[i-j]=insert_element
			i+=11
		i+=1
	return input_data 




def dothis(data):
	toedit1=data
	edited1=list(map(decrease_in_direction,toedit1.ranges[:]))
	toedit1_extended=increasing_array(edited1)
	toedit1.ranges=toedit1_extended
	xy_e1=getting_cordi(edited1,list(map(increase_in_direction,toedit1.ranges[:])),data.angle_increment)
	pub5=rospy.Publisher("tp5xy5",PolygonArray,queue_size=0)
	pub5.publish(xy_e1)
	pub1=rospy.Publisher("tp1",LaserScan,queue_size=10)
	pub1.publish(toedit1)
	rate=rospy.Rate(10)
	# rate.sleep()




def dothat(datta):
	toedit2 = LaserScan()
	toedit2=datta
	edited2=list(map(increase_in_direction,toedit2.ranges[:]))
	toedit2.ranges=edited2[:]
	pub3=rospy.Publisher("tp3",LaserScan,queue_size=10)
	pub3.publish(toedit2)
	rate=rospy.Rate(10)
	# rate.sleep()
	
def pose_cb(odom_data):
	global bot_x, bot_y, bot_yaw
	bot_x=odom_data.pose.pose.position.x
	bot_y=odom_data.pose.pose.position.y
	roll,pitch,yaw=tf.transformations.euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w]) 
	# print "bot yaw odom: ", bot_yaw
	bot_yaw=yaw

	
#subscriber. scan topic, takes LaserScan and passes it to callBack functions	
def expander_Attempt1():
	rospy.init_node("object_expander",anonymous=True)
	sub1=rospy.Subscriber("scan",LaserScan,dothis)
	sub2=rospy.Subscriber("scan",LaserScan,dothat)
	odom_sub = rospy.Subscriber("/odom", Odometry, pose_cb)
	rospy.spin()
	
	
if __name__=='__main__':
	print "obstacle Expander started"
	expander_Attempt1()
