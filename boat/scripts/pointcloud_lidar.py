#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import numpy as np
import math
import time



class Lidar:


    def __init__(self):
        self.points_list = []
        self.obstacle_x = 0
        self.obstacle_y = 0
        self.obstacle_d = 0
        self.state = 0
        self.theta_imu = 0
        self.tx = 0
        self.obs_detected = False
        self.ang = 1
        self.ang_change = 0
    
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback_zed_cp)
        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        self.angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)


    def callback_zed_cp2(self,ros_cloud):
        self.points_list = list(pc2.read_points(ros_cloud, skip_nans=False, field_names = ("x", "y", "z")))
        

        left,right,c = 0,0,0

        for i in len(self.points_list):
            if i[1] < -0.75:
                left += 1

            elif i[1] > 0.75:
                right += 1

            else:
                c += 1

        thresh = 10 #?

        l,r,c = left>thresh, right>thresh, center>thresh

        print(l,r,c)


    def callback_zed_cp(self,ros_cloud):
        self.points_list = list(pc2.read_points(ros_cloud, skip_nans=False, field_names = ("x", "y", "z")))
        
        x = []
        y = []
        for i in self.points_list:
            x.append(i[0])
            y.append(i[1])
        

        x = np.array(x)
        y = np.array(y)

        x = x[abs(x - np.mean(x)) < 2 * np.std(x)]
        y = y[abs(y - np.mean(y)) < 2 * np.std(y)]

        x_prom = np.mean(x)
        y_prom = np.mean(y)

        self.obstacle_x = x_prom
        self.obstacle_y = y_prom
        self.obstacle_d = (x_prom**2 + y_prom**2)**0.5

        if len(x) > 1000:
            print(len(x))
            self.obs_deteced = True

        print(x_prom,y_prom)

    def ins_pose_callback(self,pose):
        self.theta_imu = pose.theta


    def desired(self, thrust, heading):
        self.angulo_pub.publish(heading)
        self.d_thrust_pub.publish(thrust)

        
    def look_finding(self, angle):

        #.1 = 6 grados

        self.tx = 3
        delta = .0035

        if self.ang == 0:
            if self.ang_change < .15 :
                self.ang_change = self.ang_change + delta
                dh = angle + self.ang_change
                #self.theta_imu -= delta
                self.desired(self.tx, dh)
            else:
                self.ang_change = 0
                self.ang = 1

        elif self.ang == 1:
            if self.ang_change < .3 :
                self.ang_change = self.ang_change + delta
                #self.theta_imu -= delta
                dh = angle - self.ang_change
                self.desired(self.tx, dh)
            else:
                self.ang_change = 0
                self.ang = -1

        elif self.ang == -1:
            if self.ang_change < .3 :
                self.ang_change = self.ang_change + delta
                #self.theta_imu += delta
                dh = angle + self.ang_change
                self.desired(self.tx, dh)
            else:
                self.ang_change = 0
                self.ang = 1

        time.sleep(0.1)






if __name__ == '__main__':
    try:
        rospy.init_node('lidar_node')

        rate = rospy.Rate(10) # 10Hz
        D = Lidar()

        
        curr_angle = D.theta_imu
        while D.obs_detected == False:
            print('buscando')
            D.look_finding(curr_angle)
        
        while D.obstacle_d > 2:
            print('derecho')
            D.desired(17,D.thetha_imu)
        
        curr_angle = D.theta_imu
        
        delta = 0.035
        while D.obstacle_d < 2:
            print('derecha')
            D.desired(10, curr_angle + delta)
            delta += delta
            time.sleep(0.1)
        
        curr_angle2 = curr_angle+3.14 if curr_angle + 3.14 <= 3.14 else curr_angle + 3.14 - 6.28

        while D.theta_imu != curr_angle2: #corregir
            #girar a la izquierda avanzando
            
            while D.obstacle_d > 2:
                print('izquierda')
                D.desired(10, curr_angle - delta)
                delta += delta
                time.sleep(0.1)

           
            D.desired(5, D.theta_imu)
            time.sleep(1)
        
        print('fin')

            

            
                
        

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
