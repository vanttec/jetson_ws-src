#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "custom_msgs/ObjDetectedList.h"
#include <Eigen/Dense>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int length;
float distanceX[]={};
float Ys[]={};


void obstaculosCallback(const custom_msgs::ObjDetectedList::ConstPtr& msg)
   {
   length=0;
    length=msg->len;
    //float objects;
    //float objects=msg-> objects;
    //obj = msg->objects;
    ROS_INFO("length: %i",length);

 


    cv::Mat A;

   A = cv::Mat(1000,1000,CV_8UC1, cv::Scalar::all(0));

   //int count = 0;

    float Yactual=0;
    float Xactual = 0;
    
    ROS_INFO("length: %i",length);
    
    for (int i = 0; i<(length); i++)
    {
        
        
        Xactual=msg->objects[i].X*50+12.5;
        Yactual = msg->objects[i].Y*50+500;
        ROS_INFO("Xm: %f",msg->objects[i].X);
        ROS_INFO("Ym: %f",msg->objects[i].Y);
        ROS_INFO("Xactual: %f",Xactual);
        ROS_INFO("Yactual: %f",Yactual);
        ROS_INFO("j: %i",i);

        cv::circle( A, cv::Point( Xactual, Yactual ), 12.5, cv::Scalar( 255 ), -1, 8 ); 
        //S_INFO("Xactual: %f",Xactual);
        //S_INFO("Yactual: %f",Yactual);
        //boxes[i]=(msg->objects[i].x,msg->objects[i].y,msg->objects[i].w,msg->objects[i].h);//boxes.append([data.objects[i].x,data.objects[i].y,data.objects[i].w,data.objects[i].h]);
     }   
        
    cv::imshow("map", A);
   
      //sensor_msgs::Image map;
      //10*10metros 1pixel=2centimetros
      //map.width=500;
      //map.height=500;
      //map.encoding="mono8";
      //map.size=map.height*map.width;
      //map.step = 1;





      //map.header.stamp = ros::Time::now();

      //map.is_bigendian = false;
     // map.step = 1 * map.width;
      
        cv::waitKey(3);


    
        

    
   }



int main(int argc, char **argv)
{

    ros::init(argc, argv, "map_image_cv");

   ros::NodeHandle n;

   //ROS Publishers for each required sensor data
   ros::Subscriber obstaculos_sub = n.subscribe("objects_detected", 10, obstaculosCallback);


   ros::Rate loop_rate(10);

    


        ros::spin();

      loop_rate.sleep();
   

   return 0;
}
