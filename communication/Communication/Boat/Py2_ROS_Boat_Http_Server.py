from flask import Flask
#import rospy

app = Flask("Py2_Server")

@app.route("/")
def index():
	return 'Index Page'

@app.route("/AutonomousNavigation")
def receive_autonomous_navigation():
	return "Launching Rostopic AutonomousNavigation"

@app.route("/FindThePath")
def receive_find_the_path():
	return "Launching Rostopic FindThePath"

@app.route("/SpeedChallenge")
def receive_speed_challenge():
	return "Launching Rostopic SpeedChallenge"

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
	app.run(debug=True)