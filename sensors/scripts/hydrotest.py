#!/usr/bin/env python

import os
import time
import rospy
from std_msgs.msg import Int32
import serial

#Hydro test
class Test:
    def __init__(self):
        self.testing = True
        self.baudRate = 115200
        self.serial_port = '/dev/ttyACM0'
        self.hydro_pub = rospy.Publisher("hydro", Int32, queue_size=10)
        self.ser = serial.Serial(self.serial_port, self.baudRate)
        
    def read_data(self):
        self.arduinoData = self.ser.read(size=1)
        self.arduinoString = str(self.arduinoData)
        #print(self.arduinoString.encode('hex'))
        self.V = int(self.arduinoString.encode('hex'), 16)
        self.hydro_pub.publish(self.V)

def main():
    rospy.init_node('hydrotest', anonymous=True)
    t = Test()
    while t.testing:
        t.read_data()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
