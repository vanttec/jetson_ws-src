#!/usr/bin/env python

import os
import time
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from std_msgs.msg import Int32

#Constant Thrust and Constant Heading
#15 N and imu rads
class Test:
    def __init__(self):
        self.testing = True

        self.dt = 0
        self.dh = 0

        self.heading = 0        

        rospy.Subscriber("ins_pose", Pose2D, self.ref_callback)

        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)

    def ref_callback(self, refh):
        self.heading = refh.theta

    def desired(self, thrust, heading):
        self.dt = thrust
        self.dh = heading
        self.d_thrust_pub.publish(self.dt)
        self.d_heading_pub.publish(self.dh)

def main():
    rospy.init_node('ha', anonymous=True)
    t = Test()
    if t.testing:
        start_time = rospy.Time.now().secs           
        while (rospy.Time.now().secs - start_time) <= 1:
            t.desired(0,-2.14)
            time.sleep(0.1)
        reference_heading = t.heading        
        while (rospy.Time.now().secs - start_time) <= 40:
            t.desired(25,-2.14)
            time.sleep(0.1)
        t.desired(0,-2.14)
        t.testing = False
        rospy.logwarn("Finished")
        E.status_pub.publish(1)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
