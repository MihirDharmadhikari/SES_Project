# -*- coding: utf-8 -*-
"""
Created on Tue Apr 23 18:12:36 2019

@author: adity
"""

import numpy as np
import tf
import math
pi=math.pi
# import rospy

class convert():
    def __init__(self):
        self.init_data()
        # rospy.init_node('converter', anonymous=True)
        # self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.conv_xy)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_botpos)
    def init_data(self):
        self.scan_data=[]
        self.xy_scan_data=[]
        self.xy_after_trans=[]
        self.scan_after_trans=[]
        
        self.tf_mat=np.zeros([3,3])
        self.tf_mat[2,2]=1
        self.rot_max=np.zeros([2,2])
        self.trans_mat=np.zeros([1,2])
        
        self.bot_x=1
        self.bot_y=1
        self.bot_yaw=0
        
    def get_botpos(self, odom_data):  
        self.bot_x=odom_data.pose.pose.position.x
        self.bot_y=odom_data.pose.pose.position.y
        roll,pitch,yaw=tf.transformations.euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w]) 
        self.bot_yaw=yaw

        cos=math.cos(self.bot_yaw*pi/180)
        sin=math.sin(self.bot_yaw*pi/180)
        self.rot_max=np.array([[cos , sin],[-sin , cos]])
        self.trans_mat=np.array([[self.bot_x , self.bot_y]])
    
    def conv_xy(self, scan_d):
        """converting scan data to cordinate frame"""
        # scan_d = scan_d_ip.ranges
        for i in range(len(scan_d)):
            if scan_d[i]=="inf":
                self.xy_scan_data.append('inf')
            else:
                x_val=scan_d[i]*math.cos(i*pi/180)
                y_val=scan_d[i]*math.sin(i*pi/180)
                cordinate=[x_val,y_val]
                #to_append = np.array(cordinate)
                self.xy_scan_data.append(cordinate)
        print("polar to cordinate--->")
        print(self.xy_scan_data)
        print("---------------------------------------------")

        # self.tranform()
    
    def tranform(self):
        """multiplying with transform matrix"""
        for i in range (len(self.xy_scan_data)):
            if self.xy_scan_data[i]!="inf":
                af_rot=np.reshape(self.xy_scan_data[i],[1,2])
                af_rot=np.dot(af_rot,self.rot_max)
                af_trans=af_rot+self.trans_mat
                self.xy_after_trans.append(af_trans)
            else:
                self.xy_after_trans.append('inf')
        print("After transformation--->")
        print(self.xy_after_trans)
        print("---------------------------------------------")

    def polar_scan(self):
        for i in range(len(self.xy_after_trans)):
            if self.xy_after_trans[i] != 'inf':
                #print(self.xy_after_trans[i])    
                a=math.sqrt((self.xy_after_trans[i][0][0])**2 + (self.xy_after_trans[i][0][1])**2)
                self.scan_after_trans.append(a)
            else:
                self.scan_after_trans.append('inf')
        print("Polar after transformation--->")
        print(self.scan_after_trans)
        print("---------------------------------------------")
        return self.scan_after_trans
        
if __name__=="__main__":
	a=convert()
	# l=[1, 'inf', 'inf', 2, 3, 4, 5, 1, 1, 1, 1, 'inf', 'inf']
	# a.conv_xy(l)
	# a.get_botpos(1)
	# a.tranform()
	# up_scan=a.polar_scan()
 
                
            