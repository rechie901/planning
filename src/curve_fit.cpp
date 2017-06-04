#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>
#include "spline.h"


// #define max_accel 100   //percent throttle
// #define PI 3.14159
// #define max_speed 20   //kmph
// #define l 2.1          // wheelbase length in meters

using ecl::SmoothLinearSpline;
using ecl::CubicSpline;
using ecl::TensionSpline;
using ecl::Array;



int main(int argc, char **argv){
	ros::init(argc, argv, "spline_node");
  	ros::NodeHandle node;
	CubicSpline cubic1, cubic2;
	//SmoothLinearSpline spline1, spline2;
	Array<double> array1;
	Array<double> array2;
	Array<double> array3;
	array1.resize(4); // x
	array2.resize(4); // y
	array3.resize(4);  // t 
	array1<<0, 30, 40, 40;
	array2 << 0, 0, 10, 50;
	array3 << 0 , 33.33, 66.66, 100;


	cubic1 = CubicSpline::Natural(array3, array1);
	cubic2 = CubicSpline::Natural(array3, array2);

	ros::Publisher pub = node.advertise<nav_msgs::Path>("path", 1);
	ros::Publisher pub2 = node.advertise<nav_msgs::Path>("expected_path", 1);
	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
	ros::Publisher vis_pub2 = node.advertise<visualization_msgs::Marker>( "visualization_marker_text", 1 );
	nav_msgs::Path path,expected;
	path.header.frame_id = "odom";
	path.header.stamp = ros::Time();
	expected.header.frame_id = "odom";
	expected.header.stamp = ros::Time();

	std::vector<double> X(5), Y(5), T(5);
   	X[0]=0; X[1]=30; X[2]=40; X[3]=30; X[4]=10;
   	Y[0]=0; Y[1]=0; Y[2]=10; Y[3]=30; Y[4]=50;
   	T[0]=0; T[1]=25; T[2]=50; T[3]=75; T[4]=99;
   
   	tk::spline s1,s2;
   	s1.set_points(T,X);
   	s2.set_points(T,Y);    // currently it is required that X is already sorted
   	float length = 0;
   	float del_theta = 0, avg_delta =0, last_heading, theta;

	path.poses.resize(100);
	for(int i=0 ; i < 100; i++){
		double t = i;
		path.poses[i].pose.position.x = s1(t);//cubic1(t);
		path.poses[i].pose.position.y = s2(t);//cubic2(t);
		if(i>0){
			length = length + sqrt((path.poses[i].pose.position.x-path.poses[i-1].pose.position.x)*(path.poses[i].pose.position.x-path.poses[i-1].pose.position.x) + (path.poses[i].pose.position.y-path.poses[i-1].pose.position.y)*(path.poses[i].pose.position.y-path.poses[i-1].pose.position.y));
			if((path.poses[i].pose.position.x-path.poses[i-1].pose.position.x) != 0)
				theta = fabs(atan((path.poses[i].pose.position.y-path.poses[i-1].pose.position.y)/(path.poses[i].pose.position.x-path.poses[i-1].pose.position.x)));
			else
				theta = theta;
			// if(i > 37)
			// 	//std::cout <<"numerator : "<< (path.poses[i].pose.position.y-path.poses[i-1].pose.position.y)<< " denominator :  " << (path.poses[i].pose.position.x-path.poses[i-1].pose.position.x)<<" tan  "<< theta << " last_heading "<< last_heading<< std::endl;
			del_theta = theta - last_heading;
			// if(del_theta > 0.1){

			}
			avg_delta = avg_delta + fabs(del_theta);		
		std::cout << " " <<t<<" del_theta "<< del_theta*180/3.14 << " length" << length<< std::endl;
		last_heading = theta;
	}
	std::cout << " avg_delta " << avg_delta*3.14/(length*180)<<std::endl;
	// for(int i ; i < curve_vec.size(); i++){
	// 	expected.poses[i].pose.position.x = curve_vec[i].kx;
	// 	expected.poses[i].pose.position.y = curve_vec[i].ky;
	// }

	ros::Rate loop_rate(100);
	while(ros::ok()){

		pub.publish(path);
		loop_rate.sleep();
		ros::spinOnce();
		
	}
	
	return 0;
}