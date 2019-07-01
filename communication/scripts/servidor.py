#!/usr/bin/env python

import socket
import sys
from nmeaserver import formatter

from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import datetime


import rospy
import subprocess
import os



class Servidor:
    def __init__(self):
        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        self.theta_imu = 0
        self.lat = 0
        self.lon = 0
        

        


        


    def ins_pose_callback(self,pose):
        self.theta_imu = pose.theta
        self.lat = pose.x
        self.lon = pose.y
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        HOST, PORT = "localhost", 9000 # for robot.server in the competition
    # HOST, PORT = "localhost", 9000 # for the router
        server_address = (HOST, PORT)
        print >>sys.stderr, 'connecting to %s port %s' % server_address
        sock.connect(server_address)
        try:
        # Send heartbeat

            current_time = datetime.datetime.now()

            date = str(current_time.day) + '0' +str(current_time.month) + str(current_time.year)[-2:]

            hour = str(current_time.hour) if len(str(current_time.hour)) == 2 else '0'+str(current_time.hour)
            sec = str(current_time.second) if len(str(current_time.second)) == 2 else '0'+str(current_time.second)
            minu = str(current_time.minute) if len(str(current_time.minute)) == 2 else '0'+str(current_time.minute)

            date_time = hour + minu + sec

            print(date, date_time)

            message = formatter.format(
                'RBHRB,'+ date + ',' + date_time + ',' + str(abs(self.lat)) + ',N,'+ str(abs(self.lon)) +',W,VTEC,')
            print >>sys.stderr, 'sending "%s"' % message
            sock.sendall(message)
         
        # here goes: retrieve message and check status of the connection
        # and reconnect automatically

        finally:
            print >>sys.stderr, 'closing socket'
            sock.close()


if __name__ == '__main__':
    rospy.init_node('servidor', anonymous=True)
    rate = rospy.Rate(10)
    S = Servidor()
    rospy.spin()
