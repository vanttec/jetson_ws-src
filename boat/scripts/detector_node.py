#!/usr/bin/env python
"""
    @modified: Wed Jan 30, 2019
    @author: Ingrid Navarro
    @file: detector_node.py
    @version: 1.0
    @brief:
        This code implements a ROS node to perform object detection and classification
        using YOLO detection frameworks. Node receives frames from another node and
        publishes the coordinates of the detected objects.
    @requirements:
        Tested on python2.7 and python3.6.
        OpenCV version 3.4+ (because it uses the "dnn" module).
        Cuda version 8.0
        Tested on ROS Kinetic.
        Tested on Ubuntu 16.04 LTS
"""
from detection.detector import Detector
from std_msgs.msg import String
from imutils.video import VideoStream
from imutils.video import FPS

from custom_msgs.srv import ColorDeImagen
from custom_msgs.srv import DistanceCal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList

import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import imutils
import argparse
import numpy as np
import time
import rospy
import cv2
import math

# Parse arguments
#ap = argparse.ArgumentParser()
#ap.add_argument('--config', required=True, help = 'Path to yolo config file')
#ap.add_argument('--weights', required=True, help = 'Path to yolo pre-trained weights')
#ap.add_argument('--classes', required=True, help = 'Path to text file containing class names')
#ap.add_argument('--video', required=True, help = 'Path to the video' )
#args = ap.parse_args()

bridge = CvBridge()

class Color():
    BLUE  = '\033[94m'
    GREEN = '\033[92m'
    RED  = '\033[91m'
    DONE  = '\033[0m'

def add_brightness(image):
    image_HLS = cv2.cvtColor(image,cv2.COLOR_RGB2HLS) ## Conversion to HLS
    image_HLS = np.array(image_HLS, dtype = np.float64)
    random_brightness_coefficient = 1.5 ## generates value between 0.5 and 1.5
    image_HLS[:,:,1] = image_HLS[:,:,1]*random_brightness_coefficient ## scale pixel values up or down for channel 1(Lightness)
    image_HLS[:,:,1][image_HLS[:,:,1]>255]  = 255 ##Sets all values above 255 to 255
    image_HLS = np.array(image_HLS, dtype = np.uint8)
    image_RGB = cv2.cvtColor(image_HLS,cv2.COLOR_HLS2RGB) ## Conversion to RGB
    return image_RGB

def add_darkness(image):
    image_HLS = cv2.cvtColor(image,cv2.COLOR_RGB2HLS) ## Conversion to HLS
    image_HLS = np.array(image_HLS, dtype = np.float64)
    random_brightness_coefficient = 0.5 ## generates value between 0.5 and 1.5
    image_HLS[:,:,1] = image_HLS[:,:,1]*random_brightness_coefficient ## scale pixel values up or down for channel 1(Lightness)
    image_HLS[:,:,1][image_HLS[:,:,1]>255]  = 255 ##Sets all values above 255 to 255
    image_HLS = np.array(image_HLS, dtype = np.uint8)
    image_RGB = cv2.cvtColor(image_HLS,cv2.COLOR_HLS2RGB) ## Conversion to RGB
    return image_RGB


def calculate_distance(img,x,y,w,h):
	""" Calculates distance using get_distance service """
	global bridge

	img = bridge.cv2_to_imgmsg(img)
	rospy.wait_for_service("/get_distance")

	service = rospy.ServiceProxy("/get_distance", DistanceCal)
	dis = service(img,x,y,w,h)

	return dis



def calculate_color(img,x,y,w,h):
	""" Calculates distance using get_distance service """
	
	global bridge

	img = bridge.cv2_to_imgmsg(img, encoding = "bgr8")
	rospy.wait_for_service("/get_color")

	service = rospy.ServiceProxy("/get_color", ColorDeImagen)
	color = service(img,x,y,w,h)

	return color
	

def send_message(color, msg):
    """ Publish message to ros node. """
    msg = color + msg + Color.DONE
    rospy.loginfo(msg)
    #detector_pub.publish(msg)

def detect():
    """ Performs object detection and publishes coordinates. """
    
    #global image and depth frames from ZED camera
    global image
    global points_list

    points_list = [[0,0,0]]*921600
    image = np.zeros((560,1000,3),np.uint8)
    # Initialize detector
    send_message(Color.GREEN, "[INFO] Initializing TinyYOLOv3 detector.")
    det = Detector("/home/nvidia/catkin_ws/src/boat/scripts/vantec-config/tiny3.cfg", "/home/nvidia/catkin_ws/src/boat/scripts/vantec-config/tiny3_68000.weights", "/home/nvidia/catkin_ws/src/boat/scripts/vantec-config/obj.names")
    (H, W) = (None, None)

    # Load model
    send_message(Color.GREEN, "[INFO] Loading network model.")
    net = det.load_model()

    # Initilialize Video Stream
    send_message(Color.GREEN, "[INFO] Starting video stream.")
    
    counter = 0
    dets = 0
    nondets = 0
    detect = True
    fps = FPS().start()
    boxes, confidences, indices, cls_ids, colors, ids, distances = [], [], [], [], [], [], []


    ret = True
    while not rospy.is_shutdown():
        # Grab next frame
	
        #ret, frame = video.read()
        frame = image

        color = ""
        diststring = ""

        ##AQUI SE MODIFICA EL VIDEO

        #frame = add_brightness(frame)
        #frame = add_darkness(frame)

        if cv2.waitKey(1) & 0xFF == ord ('q'):
            send_message(Color.RED, "[DONE] Quitting program.")
            break

        print(frame.shape)
        frame = imutils.resize(frame, width=1000)



        (H, W) = frame.shape[:2]
        if det.get_w() is None or det.get_h() is None:
            det.set_h(H)
            det.set_w(W)

        # Perform detection
        
        detect = True
        dets += 1
        # Get bounding boxes, condifences, indices and class IDs
        boxes, confidences, indices, cls_ids = det.get_detections(net, frame)
        # Publish detections
        
        det_str = "Det: {}, BBoxes {}, Colors {}, Distance {}".format(dets, boxes, colors, distances)
        send_message(Color.BLUE, det_str)

        

        # If there were any previous detections, draw them

        colors = []
        distances = []
        obj_list = ObjDetectedList()
        len_list = 0
		
        for ix in indices:
			i = ix[0]

			box = boxes[i]
			x, y, w, h = box
			x, y, w, h = int(x), int(y), int(w), int(h)

			if detect == True:
				color = calculate_color(frame,x,y,h,w)
				p1= int((x+w/2)*1.28)
				p2= int((y+h/2)*1.28)
				
				dist = points_list[(p1+p2*1280)][0]

				if (dist < .30):
					diststring = "OUT OF RANGE"
				else:
					diststring = str(dist) + " m"
				
				color = str(color.color)
				colors.append(color)
				distances.append(dist)
				
                
				
				
				obj = ObjDetected()
				print(p1,p2)
				obj.x = x
				obj.y = y
				obj.h = h
				obj.w = w
				obj.X = points_list[(p1+p2*1280)][0]
				obj.Y = points_list[(p1+p2*1280)][1]
				obj.color = color
				obj.clase = 'bouy' if cls_ids[i] == 0 else 'marker'
				len_list += 1
				obj_list.objects.append(obj)

							
				det.draw_prediction(frame, cls_ids[i], confidences[i], color,diststring, x, y, x+w, y+h)
		
			
				
			
		
        fps.update()
        obj_list.len = len_list
        detector_pub.publish(obj_list)
        cv2.line(frame, (500,560), (500,0), (255,0,0))
        
        fps.stop()

        info = [
            ("Detects: ", dets),
            ("No detects: ", nondets),
            ("FPS", "{:.2F}".format(fps.fps())),
        ]
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, det.get_h() - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Show current frame

        
        #cv2.imshow("depth", depth_frame)
        cv2.imshow("Frame", frame)
       	#while cv2.waitKey(0) and 0xFF == ord('q'):      
        rate.sleep()
        

def callback_zed_img(img):
	""" ZED rect_image callback"""
	
	global bridge
	global image
	image = bridge.imgmsg_to_cv2(img)
	#cv2.imshow("image", image)
	#cv2.waitKey(0)



def callback_zed_cp(ros_cloud):
    
    global points_list
    


    points_list = list(pc2.read_points(ros_cloud, skip_nans=False, field_names = ("x", "y", "z")))
    
   

if __name__ == '__main__':
    try:

    	rospy.Subscriber("/zed/rgb/image_rect_color", Image, callback_zed_img)
    	#rospy.Subscriber("/zed/depth/depth_registered", Image, callback_zed_depth)
    	rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback_zed_cp)
    	

        # Create publisher
        detector_pub = rospy.Publisher('/objects_detected', ObjDetectedList, queue_size=10)
        #detector_pub = rospy.Publisher('detections', String, queue_size=10)
        rospy.init_node('detector')

        rate = rospy.Rate(10) # 10Hz

        detect()
    except rospy.ROSInterruptException:
        pass
