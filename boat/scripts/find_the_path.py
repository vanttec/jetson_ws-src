#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import MultiArrayLayout

from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList

import numpy as np



class Find_The_Path:
    def __init__(self):

        self.counter = 0
        self.path_list = []
        self.obj_list = []

        rospy.Subscriber("path_waypoints", Float32MultiArray, self.path_callback)
        rospy.Subscriber('/objects_detected', ObjDetectedList, self.objs_callback)


        self.angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)



    def path_callback(self,msg):
        self.path_list.append({'obj_c': msg.data[0], 'size' : msg.layout.data_offset, 'path': list(reversed(list(msg.data[1:])))})


    def objs_callback(self,data):
        print("a")
        self.obj_list = []
        for i in range(data.len):
            self.obj_list.append({'X' : data.objects[i].X, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})

    def desired(self, thrust, heading):
        self.angulo_pub.publish(heading)
        self.d_thrust_pub.publish(thrust)



if __name__ == '__main__':
    rospy.init_node('find_the_path', anonymous=True)
    rate = rospy.Rate(10)

    E = Find_The_Path()

    while len(E.path_list) <= 5:
        pass
        
    sortList = sorted(E.path_list, key=lambda k: k['obj_c'],reverse=True)
    sortList2 = [a for a in sortList if a['obj_c'] == sortList[0]['obj_c']]
    sortList2 = sorted(sortList2, key=lambda k: k['size'],reverse=False)

    dst_marker = [a['path'][-2:] for a in sortList2]
    dst_marker_prom = np.mean(dst_marker, axis = 0)

    print(dst_marker_prom)


    sortList2[0]['path'][-2:] = dst_marker_prom
    #print(dst_marker)

    for x in sortList: print(x['obj_c'], x['size'])
    print('------------------------------------------------')
    for x in sortList2: print(x['obj_c'], x['size'])
    print('------------------------------------------------')
    print(sortList2[0])
    print(sortList2[0]['obj_c'],sortList2[0]['size'] )

    while True:
        for idx,item in enumerate(E.obj_list):
            if item['class'] == 'marker':
                marker_X = item['X']
                marker_Y = item['Y']
                break



