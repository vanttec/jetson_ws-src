from detection.detector import Detector 
from std_msgs.msg import String
from imutils.video import VideoStream
from imutils.video import FPS

from custom_msgs.srv import ColorDeImagen
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import imutils
import argparse
import numpy as np 
import time
import rospy
import cv2
import math

bridge = CvBridge()

def callback_zed_img(img):

	global bridge
	image = bridge.imgmsg_to_cv2(img,'bgr8')
	cv2.imshow("hola", image)
	cv2.waitKey(3)
	rospy.loginfo("got image")
	
	

if __name__ == '__main__':
    try:
    
    	rospy.Subscriber("/zed/rgb/image_rect_color", Image, callback_zed_img)
        # Create publisher 

        rospy.init_node('detector')
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
