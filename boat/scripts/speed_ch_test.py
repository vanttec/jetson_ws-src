#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Int32


from geometry_msgs.msg import Pose2D

from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

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
        rospy.Subscriber('/objects_detected', ObjDetectedList, self.callback)
        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        self.path_pub = rospy.Publisher('waypoints', Float32MultiArray, queue_size=10)
        self.angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)


    def status_callback(self,msg):
        self.status = msg.data

    def curr_time(self):
        curTime = self.current()
        difTime = curTime - self.InitTime
        realTime = difTime/float(1000)
        timer = '{:.3f}'.format(realTime)
        return float(timer)


    def ins_pose_callback(self, data):
        self.theta_imu = data.theta
        #self.theta_imu = 0
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



    def punto_medio(self):

        distances = []
        Ys = []
        for i in range(len(self.obj_list)):
            distances.append(self.obj_list[i]['X'])
            Ys.append(self.obj_list[i]['Y'])

        idx1 = np.argsort(distances)[0]
        idx2 = np.argsort(distances)[1]
        
        z1 = distances[idx1]
        
        y1 = -1*Ys[idx1]

        z2 = distances[idx2]
        
        y2 = -1*Ys[idx2]
        
        zc = min([z1,z2]) + abs(z1 - z2)/2
        yc = min([y1,y2]) + abs(y1 - y2)/2
        self.distance = zc
        offset = .55
        
        yc = 0.00001 if yc == 0 else yc

        ang = math.atan((zc+offset)/yc)
        ang_rad = ang
        ang = math.degrees(ang)
        

        
        if ang < 0:
            ang = -1*(ang + 90)
        else:
            ang = 90 - ang
        
        ang_rad = math.radians(ang)
        
        
        #print(ang)
        self.ang = -1 if ang_rad < 0 else 1

        ang_final = ang_rad + self.theta_imu
        

        #self.angulo_pub.publish(ang_final)
        self.tx = 10
        if self.distance < 7:
            self.tx = 7
        if self.distance < 0.5:
            self.tx = 0

        #self.d_thrust_pub.publish(self.tx)

        if abs(z1 - z2) <= 5:
            self.desired(self.tx, ang_final)

        
    def waypoints_vuelta(self,v_x,v_y):
        
            print('Empezo waypoints')
            
            radio = 2

            w1 = [v_x,v_y+radio]
            w2 = [v_x+radio,v_y]
            w3 = [v_x,v_y-radio]


            obj = Float32MultiArray()
            obj.layout.data_offset = 9
            
            obj.data = [(self.gps_point_trans(w1[0],w1[1]))[0],(self.gps_point_trans(w1[0],w1[1]))[1],(self.gps_point_trans(w2[0],w2[1]))[0],(self.gps_point_trans(w2[0],w2[1]))[1],(self.gps_point_trans(w3[0],w3[1]))[0],(self.gps_point_trans(w3[0],w3[1]))[1],self.start_gps[0],self.start_gps[1],1]

            print(obj.data)
            self.path_pub.publish(obj)

            print('Termino waypoints')


    def callback(self,data):
        self.obj_list = []
        for i in range(data.len):
            if data.objects[i].X < 4:
                self.obj_list.append({'X' : data.objects[i].X, 'Y' : -1*data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})

    def straight(self):
        self.tx = 20
        self.desired(self.tx, self.theta_imu)

    def look_finding(self):

        #.1 = 6 grados

        self.tx = 1
        delta = .1

        if self.ang == 1:
            if self.ang_change < .15 :
                self.ang_change = self.ang_change + delta
                self.desired(self.tx, self.theta_imu - delta)
            else:
                self.ang_change = 0
                self.ang = -1

        else:
            if self.ang_change < .15 :
                self.ang_change = self.ang_change + delta
                self.desired(self.tx, self.theta_imu + delta)
            else:
                self.ang_change = 0
                self.ang = 1

    def finding_gate(self):
        self.tx = 0.5
        self.desired(self.tx, self.theta_imu + .1)

    def desired(self, thrust, heading):
        self.angulo_pub.publish(heading)
        self.d_thrust_pub.publish(thrust)


    
if __name__ == '__main__':

    rospy.init_node('Speed_Challenge_Test', anonymous=True)
    rate = rospy.Rate(10)
    
    E = Speed_Challenge()
    while E.activated :
        
        
        if E.state == 0:
            print(E.state)
            obj_list_curr = E.obj_list
            if (len(obj_list_curr) == 1) and (str(obj_list_curr[0]['class']) == 'bouy'):
                v_x = obj_list_curr[0]['X']
                v_y = obj_list_curr[0]['Y']
                E.start_gps = [E.lat,E.lon]
                E.state = 1
                
        if E.state == 1:
            print(E.state)
            E.waypoints_vuelta(v_x,v_y)
            E.state = 2
            
        

    #rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback_zed_cp)
    rospy.spin()
