#!/usr/bin/env python 


import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from custom_msgs.srv import CamaraImage
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def get_img():
	global bridge
	rospy.wait_for_service("/get_camara_image")
	try:
		service = rospy.ServiceProxy("/get_camara_image", CamaraImage)
		msg = service(1)
		
		rospy.loginfo("imagen recibidas")
		img = msg.img_right
		img_r = bridge.imgmsg_to_cv2(img, "bgr8")

		
		cv2.imshow(img_r)
		
		#pub.publish()
		#pub2.publish()
		
		rospy.loginfo("imagen publicadas")

	except rospy.ServiceException as e:
		rospy.logerr(e)


if __name__ == '__main__':
	
	
	rospy.init_node('prueba', anonymous = True)
	

	
	pub = rospy.Publisher("/imagen_modificada", Image, queue_size = 10)
	pub2 = rospy.Publisher("/imagen_modificada2", Image, queue_size = 10)
	
	
	get_img()
				
	rate = rospy.Rate(5)	
	
	rate.sleep()
		
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")

