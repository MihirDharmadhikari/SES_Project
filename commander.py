#! /usr/bin/env python
__author__ = "Atharv"

import rospy
import math
import dynamic_rrt_integration as dri
# from obstacle_expander.msg import *
from jsk_recognition_msgs.msg import *

from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import LineString

from std_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion


class Current():
    """
    Class for current status of bot
    """

    def __init__(self):
        self.curr_pos = [0, 0]
        self.obstacle_list = []
        self.path = []
        self.rel_path=[]
        self.curr_target = []
        self.prev_target = [0, 0]
        self.target_changed = False
        self.gp_counter=0
        self.is_rotating=0
        self.curr_ang=0
        self.goal_ang=0

        
        self.path_pub = rospy.Publisher("final_path", Float32MultiArray, queue_size = 10)
        self.odometry_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
        self.obstacle_sub = rospy.Subscriber("tp5xy5", PolygonArray, self.update_obst_list)
        self.gp_sub = rospy.Subscriber("/Global_Waypoint", Float32MultiArray, self.gb_path)
        # self.main()
        

    def odom_update(self, data):
        """Update current position of bot."""
        self.curr_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
        roll,pitch,yaw=euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.curr_ang=yaw

    def target_reached(self):
        '''checks if bot reached target'''
        if((self.curr_pos[0]-self.rel_path[self.gp_counter][0])**2 + (self.curr_pos[1]-self.rel_path[self.gp_counter][1])**2)<0.05:
            print("reached-->"+str(self.curr_pos[0])+","+str(self.curr_pos[1]))
            print self.gp_counter
            if((len(self.rel_path))>(self.gp_counter+1)):
                self.gp_counter+=1



    def gb_path(self,data):
        '''Plans locally between global path.'''
        self.curr_target = data.data
        if(self.curr_target != self.prev_target):
            self.main_response()
        else:
            self.target_changed = False
            self.dynamic_caller()

    def main_response(self):
       	"""Updates goal position and calls rrt."""
        self.rel_path = dri.rrtst.do_RRT(obstacleList2=self.obstacle_list, show_animation = False, start_point_coors = self.curr_pos, end_point_coors = self.curr_target)
        self.prev_target = self.curr_target
        self.target_changed = True
        self.gp_counter=0
        print("global update")
        self.dynamic_caller()
        

    def update_obst_list(self, data):
        """Create obstacle list."""
        self.obstacle_list = []
        for i in data.polygons:
            points_list = [(j.x, j.y) for j in i.polygon.points]
            # print points_list
            self.obstacle_list.append(Polygon(points_list))
        print self.obstacle_list


    def dynamic_caller(self):
        """Call dynamic checker while en route"""
        print self.obstacle_list
        if not self.target_changed:
           dynamic_rel_path= dri.dynamic_rrt(start = self.curr_pos, end =self.curr_target, path = self.rel_path, obstacle_list = self.obstacle_list)
        if(self.rel_path != dynamic_rel_path):
           self.rel_path = dynamic_rel_path
           self.gp_counter=0
        print self.rel_path
        self.target_reached()
        dat = Float32MultiArray()
        dat.data = self.rel_path[self.gp_counter] 
        self.path_pub.publish(dat)
        


def main(self):
    rospy.init_node("commander", anonymous=True)
    Current()
    rospy.spin()


if __name__ == '__main__':
    main()