#include "ros/ros.h"
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include "custom_msgs/ObjDetectedList.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"


#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>

#include "planning.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
using namespace cv;

int length;

void MatType( Mat inputMat )
{
    int inttype = inputMat.type();

    string r, a;
    uchar depth = inttype & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (inttype >> CV_CN_SHIFT);
    switch ( depth ) {
        case CV_8U:  r = "8U";   a = "Mat.at<uchar>(y,x)"; break;  
        case CV_8S:  r = "8S";   a = "Mat.at<schar>(y,x)"; break;  
        case CV_16U: r = "16U";  a = "Mat.at<ushort>(y,x)"; break; 
        case CV_16S: r = "16S";  a = "Mat.at<short>(y,x)"; break; 
        case CV_32S: r = "32S";  a = "Mat.at<int>(y,x)"; break; 
        case CV_32F: r = "32F";  a = "Mat.at<float>(y,x)"; break; 
        case CV_64F: r = "64F";  a = "Mat.at<double>(y,x)"; break; 
        default:     r = "User"; a = "Mat.at<UKNOWN>(y,x)"; break; 
    }   
    r += "C";
    r += (chans+'0');
    cout << "Mat is of type " << r << " and should be accessed with " << a << endl;

}


#define DEBUG 0


class path_map{
public:
	path_map(){
		ros::NodeHandle n;
		pub = n.advertise<std_msgs::Float32MultiArray>("path_waypoints", 10);

   //ROS Publishers for each required sensor data
   		obstaculos_sub = n.subscribe("objects_detected", 10, &path_map::obstaculosCallback, this);

	}
	void obstaculosCallback(const custom_msgs::ObjDetectedList::ConstPtr& msg ) {
		//Parse arguments

		length=0;
	    length=msg->len;


	    ROS_INFO("length: %i",length);

	   Mat A;

	   A = cv::Mat(1000,1000,CV_8UC3, cv::Scalar::all(255));

	   //int count = 0;

	    float Yactual=0;
	    float Xactual = 0;
	    int X_marker = 80;
	    int Y_marker = 50;
	    
	    ROS_INFO("length: %i",length);
	    
	    for (int i = 0; i<(length); i++)
	    {
	        
	        if(msg->objects[i].clase == "marker" && msg->objects[i].X <= 8){
	        	X_marker = msg->objects[i].X*50+12.5;
	        	Y_marker = msg->objects[i].Y*-50+500;
	        }
	        else{
	        	Xactual=msg->objects[i].X*50+12.5;
		        Yactual = msg->objects[i].Y*-50+500;
		        ROS_INFO("Xm: %f",msg->objects[i].X);
		        ROS_INFO("Ym: %f",msg->objects[i].Y);
		        ROS_INFO("Xactual: %f",Xactual);
		        ROS_INFO("Yactual: %f",Yactual);
		        ROS_INFO("j: %i",i);

		        cv::circle( A, cv::Point( Xactual, Yactual ), 12.5, cv::Vec3b(0, 0, 0), -1, 8 );
	        }
	        
	         
	        //S_INFO("Xactual: %f",Xactual);
	        //S_INFO("Yactual: %f",Yactual);
	        //boxes[i]=(msg->objects[i].x,msg->objects[i].y,msg->objects[i].w,msg->objects[i].h);//boxes.append([data.objects[i].x,data.objects[i].y,data.objects[i].w,data.objects[i].h]);
	     }   


		//string input_path = "/home/fitocuan/catkin_ws/src/map_image/maps/map1.png";
		//Mat input_map1 = imread(input_path, cv::IMREAD_COLOR);
		
		//MatType(A);
		//MatType(input_map1);


		

		int dilation = 10;

		// Set map 
		cout << "[INFO] Generating world from input map..." << endl;
		Planner::Map map;
		map.set_input_map(A);

		

		map.create_obstacle_map(dilation);

		

		map.generate_world();
		if (DEBUG) {
			Mat input_map = map.get_input_map();
			imshow("Input Map", input_map);
			Mat obstacle_map = map.get_obstacle_map();
			imshow("Obstacle Map", obstacle_map);
			map.print_world();
			waitKey(0);
		}


		cout << "[DONE]" << endl;

		// Set source and destination 
		cout << "[INFO] Setting source and destination..." << endl;
		Planner::PathGenerator astar;
		astar.set_src({50, 0});
		astar.set_dst({Y_marker, X_marker});
		cout << "[DONE]" << endl;	


		// Path planning 
		cout << "[INFO] Finding path using AStar algorithm..." << endl;
		std::stack<Planner::coord> path;

		astar.set_heuristic(Planner::Heuristic::euclidian);
		astar.set_diagonal_movement(true); // set to false if using Heuristic::manhattan
		astar.a_star_search(map.get_world(), path);
		std::stack<int> path_to_send = map.trace_path(path);

		if (path.empty()) 
			cout << "No path was found" << endl;
		else {
			int count = 0;
			std_msgs::Float32MultiArray path_msg;
			path_msg.data.push_back(length);

			while(!path_to_send.empty()) {
				count++;
				path_msg.data.push_back(path_to_send.top());
				path_to_send.pop();
			}
			path_msg.data.push_back(length);
			path_msg.layout.data_offset = count;
			pub.publish(path_msg);

			map.print_world();
			Mat final_map = map.get_input_map();
			imshow("Final Map", final_map);
			waitKey(3);
		}
		cout << "[DONE] " << endl;
	
	}

protected:
	ros::Subscriber obstaculos_sub;
	ros::Publisher pub;


};




int main(int argc, char **argv)
{

   ros::init(argc, argv, "path_finder");

  	
   path_map ls;

   ros::Rate loop_rate(10);

    


    ros::spin();

    //loop_rate.sleep();
   

   return 0;
}