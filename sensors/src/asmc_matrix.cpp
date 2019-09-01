#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

//Thruster outputs
float Tstbd = 0;
float Tport = 0;

//Sensor feedback
float theta = 0;
float u = 0;
float v = 0;
float r = 0;



//Tracking variables
float u_d = 0;
float psi_d = 0;

//Auxiliry variables
float u_line = 0;
float u_last = 0;
float u_d_line = 0;
float u_d_last = 0;
//float u_d_dot = 0; surge speed derivative, not necessary



void dspeed_callback(const std_msgs::Float64::ConstPtr& ud)
{
  u_d = ud->data;
}

void dheading_callback(const std_msgs::Float64::ConstPtr& psid)
{
  psi_d = psid->data;
}

void ins_callback(const geometry_msgs::Pose2D::ConstPtr& ins)
{
  theta = ins->theta;
}

void vel_callback(const geometry_msgs::Vector3::ConstPtr& vel)
{
  u = vel->x;
  v = vel->y; 
  r = vel->z;
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "asmc_m");

    ros::NodeHandle n;

  //ROS Publishers for each required sensor data
  ros::Publisher right_thruster_pub = n.advertise<std_msgs::Float64>("right_thruster", 1000);
  ros::Publisher left_thruster_pub = n.advertise<std_msgs::Float64>("left_thruster", 1000);
  ros::Publisher speed_gain_pub = n.advertise<std_msgs::Float64>("speed_gain", 1000);
  ros::Publisher heading_gain_pub = n.advertise<std_msgs::Float64>("heading_gain", 1000);

  ros::Subscriber desired_speed_sub = n.subscribe("desired_speed", 1000, dspeed_callback);
  ros::Subscriber desired_heading_sub = n.subscribe("desired_heading", 1000, dheading_callback);
  ros::Subscriber ins_pose_sub = n.subscribe("ins_pose", 1000, ins_callback);
  ros::Subscriber local_vel_sub = n.subscribe("local_vel", 1000, vel_callback);

  ros::Rate loop_rate(250);

  //Model pysical parameters
  float Xu = 0;
  float Nr = 0;
  float X_u_dot = -2.25;
  float Y_v_dot = -23.13;
  float N_r_dot = -2.79;
  float Xuu = 0;
  float m = 30;
  float Iz = 4.1;
  float B = 0.41;
  float c = 0.78;

  //Controller gains
  Vector2f k;
  k << 1.2, 0.001;
  Vector2f kmin;
  kmin << 0.01, 0.01;
  Vector2f k2;
  k2 << 1.5, 10;
  Vector2f miu;
  miu << 0.02, 0.001;
  Vector2f lambda;
  lambda << 0.02, 5;

  Vector2f T;
  T << 0, 0;
  Vector2f Ka;
  Ka << 0, 0;
  Vector2f Ka_dot;
  Ka_dot << 0, 0;
  Vector2f Ka_dot_last;
  Ka_dot_last << 0, 0;
  Vector2f ua;
  ua << 0, 0;

  while (ros::ok())
  {
      
    Xu = -25;
    Xuu = 0;
    float u_abs = abs(u);
    if (u_abs > 1.2){
      Xu = 64.55;
      Xuu = -70.92;
    }

    Nr = (-0.52)*sqrt(pow(u,2)+pow(v,2));

    Vector2f g;
    g << (1 / (m - X_u_dot)), (1 / (Iz - N_r_dot));

    Vector2f f;
    f << (((m - Y_v_dot)*v*r + (Xuu*u_abs*u + Xu*u)) / (m - X_u_dot)), (((-X_u_dot + Y_v_dot)*u*v + (Nr*r)) / (Iz - N_r_dot));

    u_line = (0.004)*(u + u_last)/2 + u_line;
    u_last = u;

    u_d_line = (0.004)*(u_d + u_d_last)/2 + u_d_line;
    //u_d_dot = (u_d - u_d_last) / 0.01; //Derivative, not necessary
    u_d_last = u_d;

    Vector2f zeta;
    zeta << u_line, theta;

    Vector2f zeta_dot;
    zeta_dot << u, r;

    Vector2f dzeta;
    dzeta << u_d_line, psi_d;

    Vector2f dzeta_dot;
    dzeta_dot << u_d, 0;

    Vector2f e = zeta - dzeta;
    //if (abs(e(0)) < 0.05){
    //    e(0) = 0;
    //}
    //if (abs(e(1)) < 0.02){
    //    e(1) = 0;
    //}
    Vector2f e_dot = zeta_dot - dzeta_dot;
    //if (abs(e_dot(0)) < 0.5){
    //    e_dot(0) = 0;
    //}
    //Vector2f s;
    //s << (e_dot(0) + lambda(0)*e(0)), (e_dot(1) + lambda(1)*e(1));
    float s0 = (e_dot(0) + (lambda(0)*e(0)));
    float s1 = (e_dot(1) + (lambda(1)*e(1)));
    
    //float s0abs = abs(s(0));
    //float s1abs = abs(s(1));
    float s0abs = abs(s0);
    float s1abs = abs(s1);
    //Vector2f s_abs;
    //s_abs << s0abs, s1abs;

    int sign0 = 0;
    int sign1 = 0;

    if (Ka(0) > kmin(0)){
        //float signvar = s_abs(0) - miu(0);
        float signvar = s0abs - miu(0);
        if (signvar == 0){
          sign0 = 0;
        }
        else {
          sign0 = copysign(1,signvar);
        }
        Ka_dot(0) = k(0) * sign0;
    }
    else{
      Ka_dot(0) = kmin(0);
    } 

    if (Ka(1) > kmin(1)){
      //float signvar = s_abs(1) - miu(1);
      float signvar = s1abs - miu(1);      
      if (signvar == 0){
        sign1 = 0;
      }
      else {
        sign1 = copysign(1,signvar);
      }
      Ka_dot(1) = k(1) * sign1;
    }
    else{
      Ka_dot(1) = kmin(1);
    }

    Ka = (0.004)*(Ka_dot + Ka_dot_last)/2 + Ka;
    Ka_dot_last = Ka_dot;

    //if (s(0) = 0){    
    if (s0 == 0){
      sign0 = 0;
    }
    else {
      //sign0 = copysign(1,s(0));
      sign0 = copysign(1,s0);
    }
    //ua(0) = (-Ka(0))*sqrt(s_abs(0))*sign0 - k2(0)*s(0);
    ua(0) = ((-Ka(0)) * sqrt(s0abs) * sign0) - (k2(0)*s0);

    //if (s(1) = 0){
    if (s1 == 0){
      sign1 = 0;
    }
    else {
      //sign1 = copysign(1,s(1));
      sign1 = copysign(1,s1);
    }
    //ua(1) = (-Ka(1))*sqrt(s_abs(1))*sign1 - k2(1)*s(1);
    ua(1) = ((-Ka(1)) * sqrt(s1abs) * sign1) - (k2(1)*s1);

    T(0) = (f(0) - (lambda(0)*e_dot(0) ) + ua(0));
    T(1) = (f(1) - (lambda(1)*e_dot(1)) + ua(1));
    T(0) = T(0) / g(0);
    T(1) = T(1) / g(1);
    
    
    if (T(0) > 73){
      T(0) = 73;
    }
    else if (T(0) < -60){
      T(0) = -60;
    }
    if (T(1) > 14){
      T(1) = 14;
    }
    else if (T(1) < -14){
      T(1) = -14;
    }
    
    if (u_d == 0){
      T(0) = 0;
      T(1) = 0;
    }

    Tport = (T(0) / 2) + (T(1) / B);
    Tstbd = (T(0) / (2*c)) - (T(1) / (B*c));

    
    if (Tstbd > 36.5){
      Tstbd = 36.5;
    }
    else if (Tstbd < -30){
      Tstbd = -30;
    }
    if (Tport > 36.5){
      Tport = 36.5;
    }
    else if (Tport < -30){
      Tport = -30;
    }

    
    //cout << Ka(0) << endl;
    //cout << s0abs << endl;
    //cout << sign0 << endl;
    //cout << s0 << endl;
    //cout << sign0 << endl;        
    //cout << ua(0) << endl;
    

//Data publishing
    std_msgs::Float64 rt;
    std_msgs::Float64 lt;
    
    std_msgs::Float64 sg;
    std_msgs::Float64 hg;

    rt.data = Tstbd;
    lt.data = Tport;
    
    sg.data = Ka(0);
    hg.data = Ka(1);

    right_thruster_pub.publish(rt);
    left_thruster_pub.publish(lt);

    speed_gain_pub.publish(sg);
    heading_gain_pub.publish(hg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
