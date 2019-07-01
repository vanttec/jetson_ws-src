#!/usr/bin/env python


from flask import Flask
from std_msgs.msg import String
from std_msgs.msg import Int32

import time
import rospy
import subprocess
import os


def status_callback(msg):
    global status
    status = msg.data
    print(status)

rospy.init_node('status', anonymous=True)
pub = rospy.Publisher('/course', String, queue_size=10)
rospy.Subscriber("status", Int32, status_callback)
#subprocess.Popen("roslaunch boat general.launch", shell = True)

app = Flask("Py2_Server")





@app.route("/")
def index():
    return 'Index Page'

@app.route("/A")
def receive_A():
    global status
    pub.publish("A")
    subprocess.Popen("rosrun boat auto_nav.py", shell = True)
    while status != 1:
        print("auto_nav working")

    nodo =  subprocess.check_output("rosnode list | grep /auto", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    
    status = 0
    print("auto_nav_finish")
    return "Launching Rostopic A"

@app.route("/SC")
def receive_SC():
    global status
    subprocess.Popen("rosrun boat speed_ch.py", shell = True)
    subprocess.Popen("rosrun sensors gps_navigation.py", shell = True)

    while status != 1:
        print("speed_ch working")

    nodo =  subprocess.check_output("rosnode list | grep /speed_ch", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    
    nodo =  subprocess.check_output("rosnode list | grep /gps_navigation", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)

    status = 0

    return "Launching Rostopic SC"

@app.route("/B")
def receive_B():
    global status
    subprocess.Popen("rosrun sensors gps_navigation.py", shell = True)
    time.sleep(1)
    cmd = '''
rostopic pub /waypoints std_msgs/Float32MultiArray "layout:
  dim:                                                     
  - label: ''
    size: 0
    stride: 0
  data_offset: 9.0                                  
data: [29.15189,-81.01653,29.15186,-81.01662,29.15192,-81.01634,29.15187,-81.01615,1]" 
'''
    subprocess.Popen(cmd, shell = True)
    while status != 1:
        pass

    nodo =  subprocess.check_output("rosnode list | grep /rostopic", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    nodo =  subprocess.check_output("rosnode list | grep /gps_navigation", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)


    return "Launching Rostopic B"

@app.route("/C")
def receive_C():
    global status
    subprocess.Popen("rosrun sensors gps_navigation.py", shell = True)
    time.sleep(1)
    cmd = '''
rostopic pub /waypoints std_msgs/Float32MultiArray "layout:
  dim:                                                     
  - label: ''
    size: 0
    stride: 0
  data_offset: 9.0                                  
data: [29.15189,-81.01653,29.15186,-81.01662,29.15192,-81.01634,29.151852,-81.0161628,1]" 
'''
    subprocess.Popen(cmd, shell = True)
    while status != 1:
        pass

    nodo =  subprocess.check_output("rosnode list | grep /rostopic", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    nodo =  subprocess.check_output("rosnode list | grep /gps_navigation", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)


    return "Launching Rostopic C"

@app.route("/K")
def receive_K():
    cmd = '''  rostopic pub /desired_thrust std_msgs/Float64 "data: 0.0" '''
    subprocess.Popen(cmd, shell = True)

    return "Launching Rostopic K"

@app.route("/D")
def receive_D():
    global status
    pub.publish("D")
    subprocess.Popen("rosrun sensors ch.py", shell = True)
    while status != 1:
        print("ch_working")

    nodo =  subprocess.check_output("rosnode list | grep /ch", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    
    status = 0
    print("ch_finish")
    return "Launching Rostopic D" 

@app.route("/G")
def receive_G():
    global status
    pub.publish("G")
    subprocess.Popen("rosrun sensors ha.py", shell = True)
    while status != 1:
        print("ha_working")

    nodo =  subprocess.check_output("rosnode list | grep /ha", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    
    status = 0
    print("ha_finish")

    return "Launching Rostopic G"
    
@app.route("/W")
def receive_W():
    global status
    subprocess.Popen("rosrun boat speed_ch_test.py", shell = True)
    subprocess.Popen("rosrun sensors gps_navigation.py", shell = True)

    while status != 1:
        pass

    nodo =  subprocess.check_output("rosnode list | grep /gps_navigation", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)


    nodo =  subprocess.check_output("rosnode list | grep /speed", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)


    status = 0


    return "Launching Rostopic W"

@app.route("/Teleop")
def receive_teleop():
    return "Launching Rostopic teleop"



if __name__ == '__main__':
    global status
    status = 0
    app.run(debug=True)
