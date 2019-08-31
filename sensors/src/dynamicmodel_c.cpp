#include <iostream>
#include "vn/sensors.h"
#include "vn/thread.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace Eigen;

float Tstbd = 0;
float Tport = 0;
float theta = 0;
float r = 0;
float Xu = 0;
float Yv = 0;
float Yr = 0;
float Nv = 0;
float Nr = 0;
float X_u_dot = -2.25;
float Y_v_dot = -23.13;
float Y_r_dot = -1.31;
float N_v_dot = -16.41;
float N_r_dot = -2.79;
float Xuu = 0;
float Yvv = -99.99;
float Yvr = -5.49;
float Yrv = -5.49;
float Yrr = -8.8;
float Nvv = -5.49;
float Nvr = -8.8;
float Nrv = -8.8;
float Nrr = -3.49;
float m = 30;
float Iz = 4.1;
float B = 0.41;
float c = 0.78;

void right_callback(const std_msgs::Float64::ConstPtr& right)
{
	Tstbd = right->data;
}

void left_callback(const std_msgs::Float64::ConstPtr& left)
{
	Tport = left->data;
}


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "dynamicmodel_c");

  	ros::NodeHandle n;

	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	const uint32_t SensorBaudrate = 115200;

	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);


	//ROS Publishers for each required sensor data
	ros::Publisher dm_pos_pub = n.advertise<geometry_msgs::Pose2D>("ins_pose", 1000);
	ros::Publisher dm_vel_pub = n.advertise<geometry_msgs::Vector3>("local_vel", 1000);

	ros::Subscriber right_thruster_sub = n.subscribe("right_thruster", 1000, right_callback);
	ros::Subscriber left_thruster_sub = n.subscribe("left_thruster", 1000, left_callback);
	//ros::Subscriber ins_pose_sub = n.subscribe("NED_pose", 1000, ins_callback);
	//ros::Subscriber local_vel_sub = n.subscribe("ins_vel", 1000, vel_callback);
	ros::Publisher ref_pub = n.advertise<geometry_msgs::Pose2D>("ref", 1000);


	ros::Rate loop_rate(250);

Vector3f upsilon; //vector upsilon = [u v r] (3 DOF local reference frame)
upsilon << 0, 0, 0;

Vector3f upsilon_dot_last;
upsilon_dot_last << 0, 0, 0;

Vector3f eta;
eta << 0, 0, 0;

Vector3f eta_dot_last;
eta_dot_last << 0, 0, 0;

	InsStateLlaRegister ref; //reference (starting) data
	ref = vs.readInsStateLla();
	geometry_msgs::Pose2D ins_ref;
	ins_ref.x = 0;
	ins_ref.y = 0;
	ins_ref.theta = (3.141592 / 180)*(ref.yawPitchRoll.x);

  while (ros::ok())
  {
  geometry_msgs::Pose2D dm_pose; //inertial navigation system pose (latitude, longitude, yaw)
  geometry_msgs::Vector3 dm_vel;

	InsStateLlaRegister ins; //Inertial Navigation System (INS) variable declaration
	ins = vs.readInsStateLla(); //Variable that reads the INS data

    theta = (3.141592 / 180)*(ins.yawPitchRoll.x); //yaw converted from degrees into radians

    r = ins.angularRate.z; //yaw rate (r)

  	Xu = -25;
  	Xuu = 0;
  	if (abs(upsilon(0)) > 1.2){
  		Xu = 64.55;
  		Xuu = -70.92;
  	}

  	Yv = 0.5*(-40*1000*abs(upsilon(1)))*(1.1+0.0045*(1.01/0.09)-0.1*(0.27/0.09)+0.016*(pow((0.27/0.09),2)));
  	Yr = 6*(-3.141592*1000)*sqrt(pow(upsilon(0),2)+pow(upsilon(1),2))*0.09*0.09*1.01;
  	Nv = 0.06*(-3.141592*1000)*sqrt(pow(upsilon(0),2)+pow(upsilon(1),2))*0.09*0.09*1.01;
  	Nr = 0.02*(-3.141592*1000)*sqrt(pow(upsilon(0),2)+pow(upsilon(1),2))*0.09*0.09*1.01*1.01;

	Vector3f eta_dot; //vector declaration of eta' = [x' y' psi'] (3 DOF global reference frame)

	Matrix3f M;
	M << m - X_u_dot, 0, 0,
		 0, m - Y_v_dot, 0 - Y_r_dot,
		 0, 0 - N_v_dot, Iz - N_r_dot;

	Vector3f T;
	T << Tport + Tstbd*c, 0, 0.5*B*(Tport - Tstbd*c);

	Matrix3f CRB;
	CRB << 0, 0, 0 - m * upsilon(1),
		 0, 0, m * upsilon(0),
		 m * upsilon(1), 0 - m * upsilon(0), 0;

	Matrix3f CA;
	CA << 0, 0, 2 * ((Y_v_dot*upsilon(1)) + ((Y_r_dot + N_v_dot)/2) * r),
		 0, 0, 0 - X_u_dot * m * upsilon(0),
		 2*(((0 - Y_v_dot) * upsilon(1)) - ((Y_r_dot+N_v_dot)/2) * r), X_u_dot * m * upsilon(0), 0;

	Matrix3f C = CRB + CA;

	Matrix3f Dl;
	Dl << 0-Xu, 0, 0,
		 0, 0-Yv, 0-Yr,
		 0, 0-Nv, 0-Nr;

	Matrix3f Dn;
	Dn << Xuu * abs(upsilon(0)), 0, 0,
		 0, Yvv * abs(upsilon(1)) + Yvr * abs(r), Yrv * abs(upsilon(1)) + Yrr * abs(r),
		 0, Nvv * abs(upsilon(1)) + Nvr * abs(r), Nrv * abs(upsilon(1)) + Nrr * abs(r);

	Matrix3f D = Dl - Dn;

	Vector3f upsilon_dot =  M.inverse()*(T - C * upsilon - D * upsilon);
	upsilon = (0.004) * (upsilon_dot + upsilon_dot_last)/2 + upsilon;
	upsilon(2) = r;
	upsilon_dot_last = upsilon_dot;

	Matrix3f J; //matrix of transformation between reference frames
	J << cos(theta), -sin(theta), 0,
		 sin(theta), cos(theta), 0,
		 0, 0, 1;

	eta_dot = J*upsilon; //transformation into local reference frame
	eta = (0.004)*(eta_dot+eta_dot_last)/2 + eta;
	eta_dot_last = eta_dot;

	float x = eta(0); //position in x
	float y = eta(1); //position in y
	dm_pose.x = x;
	dm_pose.y = y;
	dm_pose.theta = theta;

	float u = upsilon(0); //surge velocity
	float v = upsilon(1); //sway velocity
	float r = upsilon(2); //yaw rate
	dm_vel.x = u;
	dm_vel.y = v;
	dm_vel.z = r;

//Data publishing
    dm_pos_pub.publish(dm_pose);
    dm_vel_pub.publish(dm_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }

	return 0;
}
