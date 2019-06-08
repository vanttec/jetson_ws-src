#!/usr/bin/env python


import rospy
import cv2 
import numpy as np


class Color():
    BLUE  = '\033[94m'
    GREEN = '\033[92m'
    RED  = '\033[91m'
    DONE  = '\033[0m'

def send_message(color, msg):
    """ Publish message to ros node. """
    msg = color + msg + Color.DONE
    rospy.loginfo(msg)



def camaras():
	cap = cv2.VideoCapture(2)
	cap2 = cv2.VideoCapture(0)
	
	while not rospy.is_shutdown() or (cap.isOpened() and cap2.isOpened()):
		
		if cv2.waitKey(1) & 0xFF == ord ('q'):
			send_message(Color.RED, "[DONE] Quitting program.")
			break
			

		ret, frame = cap.read()
		ret2, frame2 = cap2.read()
		
		vis = np.concatenate((frame, frame2), axis=1)
		
		
		cv2.imshow("frame", vis)
		cv2.waitKey(3);
		rate.sleep()

if __name__ == '__main__':
	
	try:
		print(cv2.__version__)
		rospy.init_node('nodo_camaras')
		rate = rospy.Rate(10)
		camaras()
	except rospy.ROSInterruptException:
		pass
