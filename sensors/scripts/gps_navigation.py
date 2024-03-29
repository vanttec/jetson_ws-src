#!/usr/bin/env python

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
from std_msgs.msg import Int32


EARTH_RADIUOS = 6371000


class Navigate:
    def __init__(self):
        self.navigate = True

        self.dt = 0
        self.dh = 0
        self.distance = 0
        self.bearing = 0

        self.latitude = 0
        self.longitude = 0
        self.yaw = 0

        self.image = "000"
        self.left = 0
        self.right = 0
        self.center = 0
        self.wp_array = []
        self.wp_sent = False
        self.counter = 0

        self.choicer = 0


        self.reference_heading = 0

        self.tx = 0

        rospy.Subscriber("ins_pose", Pose2D, self.gps_callback)
        rospy.Subscriber("obstacles", String, self.obstacles_callback)
        rospy.Subscriber("waypoints", Float32MultiArray, self.waypoints_callback)

        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)
        self.pub_status = rospy.Publisher("status", Int32, queue_size=10)

    def ref_callback(self, refp):
        self.reference_heading = refp.theta

    def obstacles_callback(self, view):
        self.image = view.data

    def gps_callback(self, gps):
        self.latitude = gps.x
        self.longitude = gps.y
        self.yaw = gps.theta

    def get_degrees_and_distance_to_gps_coords(self, latitude2, longitud2):
        latitude1 = self.latitude
        longitud1 = self.longitude

        longitud_distance = (longitud1 - longitud2)
        y_distance = math.sin(longitud_distance) * math.cos(latitude2)
        x_distance = math.cos(latitude1) * math.sin(latitude2) - math.sin(latitude1) * math.cos(latitude2) * math.cos(longitud_distance)
        self.bearing = math.atan2(y_distance, x_distance)
        #self.bearing = self.bearing * (-1)
        self.deg = math.degrees(self.bearing)
        #rospy.logwarn("bearing %f", self.deg)

        phi1 = math.radians(latitude1)
        phi2 = math.radians(latitude2)
        dphi = math.radians(latitude2 - latitude1)
        dlam = math.radians(longitud2 - longitud1)
        a = math.sin(dphi/2)*math.sin(dphi/2) + math.cos(phi1)*math.cos(phi2)* math.sin(dlam/2)*math.sin(dlam/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        self.distance = 6378137 * c
        #rospy.logwarn("distance %f", self.distance)

    def avoid(self, segment, latitude2, longitud2):
        self.get_degrees_and_distance_to_gps_coords(latitude2, longitud2)

        #left, center, right
        cl = segment[0]
        cl = int(cl)
        if cl > 0:
            cl = -5
        self.left = cl

        cc = segment[1]
        cc = int(cl)
        if cc > 0:
            cc = 1
        self.center = cc

        cr = segment[2]
        cr = int(cr)
        if cr > 0:
            cr = 3
        self.right = cr

        addition = self.left + self.center + self.right 

        if self.center == 0:
            self.tx = 20
            if self.distance < 7:
                self.tx = 15
            if self.distance < 1:
                self.tx = 10
            if self.distance < 0.5:
                self.tx = 5
                self.navigate = False
            if self.distance < 0.2:
                self.tx = 0
                self.bearing = self.yaw
                self.navigate = False

        elif addition == -1 or addition == 1:
                self.tx = -5

        elif addition != 0:
            self.bearing = self.yaw + addition * 0.17
            if self.bearing > 3.14:
                self.bearing = self.bearing - 3.14
            if self.bearing < -3.14:
                self.bearing = self.bearing + 3.14

        self.desired(self.tx, self.bearing)

    def gps_point_trans(self,y,x,jaw,lat2,lon2):
        p = np.array([x,y])
        J = np.array([[math.cos(jaw), -1*math.sin(jaw)],[math.sin(jaw), math.cos(jaw)]])
        n = J.dot(p)

        phi1 = math.radians(lat2)

        latitude2  = lat2  + (n[0] / EARTH_RADIUOS) * (180 / math.pi)
        longitude2 = lon2 + (n[1] / EARTH_RADIUOS) * (180 / math.pi) / math.cos(phi1)

        return (latitude2,longitude2)

    def desired(self, thrust, heading):
        self.dh = heading
        self.dt = thrust
        self.d_heading_pub.publish(self.dh)
        self.d_thrust_pub.publish(self.dt)
        time.sleep(0.1)

    def waypoints_callback(self, msg):
        wp_t = []
        leng = (msg.layout.data_offset)
        
        for i in range(int(leng)-1):
            wp_t.append(msg.data[i])
        self.choicer = msg.data[-1]
        self.wp_array = wp_t

def main():
    rospy.init_node('GPS_navigation', anonymous=True)
    navi = Navigate()
    c = 0
    wp_t = []
    while True:
        if wp_t != navi.wp_array:
            wp_t = navi.wp_array 
            jaw = navi.yaw
            lat2 = navi.latitude
            lon2 = navi.longitude

            for i in range(0,len(wp_t),2):
                if navi.choicer == 0:
                    wp_lat, wp_lon = navi.gps_point_trans(wp_t[i],wp_t[i+1], jaw, lat2,lon2)    
                    print(wp_lat, wp_lon)

            for i in range(0,len(wp_t),2):
                if navi.choicer == 0:
                    wp_lat, wp_lon = navi.gps_point_trans(wp_t[i],wp_t[i+1], jaw, lat2,lon2)
                else:
                    wp_lat = wp_t[i]
                    wp_lon = wp_t[i+1]

                while navi.navigate:
                    
                    print(wp_lat, wp_lon)
                    print(i,i+1)
                    print(navi.choicer)
                    navi.avoid(navi.image, wp_lat, wp_lon)
                    if wp_t != navi.wp_array:
                        break

                if wp_t != navi.wp_array:
                        break
                navi.navigate = True
                rospy.logwarn("Arrived")

        navi.pub_status.publish(1)


    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
