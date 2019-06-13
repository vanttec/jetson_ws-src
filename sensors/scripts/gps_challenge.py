#!/usr/bin/env python

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import Int32

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

        self.image = "0000"
        self.farleft = 0
        self.left = 0
        self.right = 0
        self.farright = 0

        self.course = "None"

        self.reference_heading = 0

        self.tx = 0

        rospy.Subscriber("ref", Pose2D, self.ref_callback)
        rospy.Subscriber("ins_pose", Pose2D, self.gps_callback)
        rospy.Subscriber("obstacles", String, self.obstacles_callback)
        rospy.Subscriber("course", String, self.course_callback)

        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)

    def ref_callback(self, refp):
        self.reference_heading = refp.theta

    def obstacles_callback(self, view):
        self.image = view.data

    def course_callback(self, challenge):
        self.course = challenge.data

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
        self.bearing = self.bearing * (-1)
        self.deg = math.degrees(self.bearing)
        rospy.logwarn("bearing %f", self.deg)

        phi1 = math.radians(latitude1)
        phi2 = math.radians(latitude2)
        dphi = math.radians(latitude2 - latitude1)
        dlam = math.radians(longitud2 - longitud1)
        a = math.sin(dphi/2)*math.sin(dphi/2) + math.cos(phi1)*math.cos(phi2)* math.sin(dlam/2)*math.sin(dlam/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        self.distance = 6378137 * c
        rospy.logwarn("distance %f", self.distance)


    def avoid(self, segment, latitude2, longitud2):
        self.get_degrees_and_distance_to_gps_coords(latitude2, longitud2)

        #far left, close left, close right, far right
        fl = segment[0]
        fl = int(fl)
        if fl > 0:
            fl = 3
        self.farleft = fl

        cl = segment[1]
        cl = int(cl)
        if cl > 0:
            cl = -1
        self.left = cl

        cr = segment[2]
        cr = int(cr)
        if cr > 0:
            cr = 1
        self.right = cr

        fr = segment[3]
        fr = int(fr)
        if fr > 0:
            fr = -3
        self.farright = fr

        addition = self.farleft + self.left + self.right + self.farright

        if self.left == 0 and self.right == 0:
            self.tx = 20
            if self.distance < 7:
                self.tx = 10
            if self.distance < 0.5:
                self.tx = 0
                self.bearing = self.yaw
                nav.navigate = False
                self.status_pub.publish(1)

        elif addition == 0:
            if self.left != 0:
                self.tx = -5

        elif addition != 0:
            self.bearing = self.yaw + addition * 0.17
            if math.fabs(addition) < 2:
                self.bearing = self.bearing * -1


        self.desired(self.tx, self.bearing)

    def desired(self, thrust, heading):
        self.dh = heading
        self.dt = thrust
        self.d_heading_pub.publish(self.dh)
        self.d_thrust_pub.publish(self.dt)
        time.sleep(0.1)

def main():
    rospy.init_node('GPS_challenge', anonymous=True)
    navi = Navigate()
    while navi.navigate:
        if navi.course == "ASC" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "AFP" :
            lat = 25.653343
            lon = -100.291532
        elif navi.course == "AAD" :
            lat = 25.653313
            lon = -100.291219
        elif navi.course == "ARF" :
            lat = 25.653367
            lon = -100.291311
        elif navi.course == "ARD" :
            lat = 25.653304
            lon = -100.291303
        elif navi.course == "BSC" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "BFP" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "BAD" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "BRF" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "BRD" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "CSC" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "CFP" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "CAD" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "CRF" :
            lat = 25.653332
            lon = -100.291456
        elif navi.course == "CRD" :
            lat = 25.653332
            lon = -100.291456

        navi.avoid(navi.image, lat, lon)
    rospy.logwarn("Arrived")

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass