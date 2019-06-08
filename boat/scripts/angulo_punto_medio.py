#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList

from geometry_msgs.msg import Pose2D

from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

import numpy as np
import math

import matplotlib.pyplot as plt

theta_imu = 0




def punto_medio(distances, boxes,Ys):

    idx1 = np.argsort(distances)[0]
    idx2 = np.argsort(distances)[1]
    
    z1 = distances[idx1]
    
    y1 = -1*Ys[idx1]

    z2 = distances[idx2]
    
    y2 = -1*Ys[idx2]
    
    zc = min([z1,z2]) + abs(z1 - z2)/2
    yc = min([y1,y2]) + abs(y1 - y2)/2
    
    offset = .55
    
    yc = 0.00001 if yc == 0 else yc

    ang = math.atan((zc+offset)/yc)
    ang_rad = ang
    ang = math.degrees(ang)
    
    print(z1,y1)
    print(z2,y2)
    print(zc,yc)
    
    if ang < 0:
        ang = -1*(ang + 90)
    else:
        ang = 90 - ang
    
    ang_rad = math.radians(ang)
    
    
    print(ang)
    global theta_imu
    
    ang_final = ang_rad + theta_imu
    
    
    angulo_pub.publish(ang_final)
    
    plt.clf()
    plt.plot(y1,z1, 'go',markersize=5)
    plt.plot(y2,z2, 'go',markersize=5)
    plt.plot(0,0,'ro')
    plt.plot(yc,zc,'r*')
    plt.axis([-5, 5, 0, 8])
    plt.pause(0.0001)
    plt.draw()
    
points_list = []
    

def ins_pose_callback(pose):
    global theta_imu
    theta_imu = pose.theta   


def callback(data):
    global points_list
    distances = []
    boxes = []
    Ys = []
    for i in range(data.len):
        distances.append(data.objects[i].X)
        Ys.append(data.objects[i].Y)
        boxes.append([data.objects[i].x,data.objects[i].y,data.objects[i].w,data.objects[i].h])
        
        
    if data.len > 1:
        punto_medio(distances, boxes, Ys)
    



    
if __name__ == '__main__':

    rospy.init_node('pm', anonymous=True)
    #rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback_zed_cp)
    rospy.Subscriber("ins_pose", Pose2D, ins_pose_callback)
    
    
    sub = rospy.Subscriber('/objects_detected', ObjDetectedList, callback)    
    angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
    rospy.spin()


