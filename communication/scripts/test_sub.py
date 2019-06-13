#!/usr/bin/env python


from flask import Flask
from std_msgs.msg import String
from std_msgs.msg import Int32


import rospy
import subprocess
import os




def listener():
    rospy.init_node('sub_test', anonymous=True)
    rospy.Subscriber("/course", String, status_callback)
    rospy.spin()

def status_callback(msg):
    status = msg.data
    print(status)

if __name__ == '__main__':
    listener()
    

