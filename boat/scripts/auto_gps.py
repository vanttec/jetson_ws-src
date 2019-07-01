#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Int32


from geometry_msgs.msg import Pose2D

import numpy as np
import math
import time

import matplotlib.pyplot as plt

EARTH_RADIUOS = 6371000


class Speed_Challenge:
    def __init__(self):
        self.obj_list = []
        self.activated = True
        self.state = 0
        self.theta_imu = 0
        self.lat = 0
        self.lon = 0
        self.current = lambda: int(round(time.time()*1000))
        self.InitTime = self.current()
        self.ang_change = 0
        self.ang = 0
        self.start_gps = [0,0]
        self.status = 0

        
        rospy.Subscriber("status", Int32, self.status_callback)
        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        self.path_pub = rospy.Publisher('waypoints', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)


    def status_callback(self,msg):
        self.status = msg.data


    def ins_pose_callback(self, data):
        self.theta_imu = data.theta
        self.lat = data.x
        self.lon = data.y

    def gps_point_trans(self,y,x):
        p = np.array([x,y])
        J = np.array([[math.cos(self.theta_imu), -1*math.sin(self.theta_imu)],[math.sin(self.theta_imu), math.cos(self.theta_imu)]])
        n = J.dot(p)

        phi1 = math.radians(self.lat)

        latitude2  = self.lat  + (n[1] / EARTH_RADIUOS) * (180 / math.pi)
        longitude2 = self.lon + (n[0] / EARTH_RADIUOS) * (180 / math.pi) / math.cos(phi1)
        
        return latitude2,longitude2



    
if __name__ == '__main__':

    rospy.init_node('auto_gps', anonymous=True)
    rate = rospy.Rate(10)
    
    E = Speed_Challenge()
    while E.activated :
        
        
        if E.state == 0:
            print(E.state)
            E.gps_point_trans(0,20)
            E.state = 1
                
        if E.state == 1:
            print(E.state)
            E.waypoints_vuelta(v_x,v_y)
            E.state = 2
            
        

    #rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback_zed_cp)
    rospy.spin()
