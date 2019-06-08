#!/usr/bin/env python

import rospy
import cv2
from custom_msgs.srv import CamaraImage, CamaraImageResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
img_l = Image()
img_r = Image()


def image_callback_r(ros_image):

	global img_r
	global bridge

	try:

		img_r = ros_image		
				
	except CvBridgeError as e:
		print(e)
		
def image_callback_l(ros_image):

	global img_l
	global bridge

	try:

		img_l = ros_image		
				
	except CvBridgeError as e:
		print(e)



def callback_camara_imgs(sel):


	global img_l
	global img_r
	
	rospy.loginfo("Imagenes enviadas!")

	return CamaraImageResponse(	
		img_right = img_r,
		img_left = img_l
	)


if __name__ == '__main__':

	rospy.init_node('camara_image_server')
	rospy.loginfo("Node created!")
	
	image_sub_r = rospy.Subscriber("/usb_cam1/image_raw", Image, image_callback_r)
	image_sub_l = rospy.Subscriber("/usb_cam2/image_raw", Image, image_callback_l)
	
	service = rospy.Service("/get_camara_image", CamaraImage, callback_camara_imgs)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()
