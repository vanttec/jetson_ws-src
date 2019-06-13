#!/usr/bin/env python


from flask import Flask
from std_msgs.msg import String
from std_msgs.msg import Int32


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

@app.route("/B")
def receive_B():
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

    return "Launching Rostopic B"

@app.route("/C")
def receive_C():
    global status
    subprocess.Popen("rosrun sensors gps_navigation.py", shell = True)

    while status != 0:
        pass

    nodo =  subprocess.check_output("rosnode list | grep /gps_navigation", shell = True)
    cmd = "rosnode kill " + str(nodo)
    subprocess.Popen(cmd, shell = True)
    
    return "Launching Rostopic C"

@app.route("/RaiseTheFlag")
def receive_raise_the_flag():
    return "Launching Rostopic RaiseTheFlag"

@app.route("/AutomatedDocking")
def receive_automated_docking():
    return "Launching Rostopic AutomatedDocking" 
    
@app.route("/GPSNavigation")
def receive_GPS_navigation():
    return "Launching Rostopic GPSNavigation"

@app.route("/Teleop")
def receive_teleop():
    return "Launching Rostopic teleop"



if __name__ == '__main__':
    global status
    status = 0
    app.run(debug=True)






