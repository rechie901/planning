#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>
#include "spline.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>

struct state{
	float x, y, heading, steer_angle, velocity;
};
struct waypoints{
	float x, y;
};
state current_states;
state init_pos;
waypoints current_pts;
///--------- filter noise in GPS --------------------------------------------
// void filter(){
	
// }
///----------------store waypoints to txt ------------------------------------------
// void store_waypoints(std::ofstream myfile, waypoints data){
	

  	
// }

//////-------------------------- Visualization Markers ----------------------------------
int init_flag = 0;
float PI = 3.14;
void visualization(visualization_msgs::Marker::Ptr marker, float pos_x, float pos_y,tf::Quaternion q_tf, float red, float green, float blue){
	//visualization_msgs::Marker marker_goal;
	marker->header.frame_id = "odom";
	marker->header.stamp = ros::Time();
	marker->type = visualization_msgs::Marker::CUBE;
	marker->action = visualization_msgs::Marker::ADD;
	marker->pose.position.x = pos_x;
	marker->pose.position.y = pos_y;
	marker->pose.position.z = 1;
	marker->pose.orientation.x = q_tf.getX();
	marker->pose.orientation.y = q_tf.getY();
	marker->pose.orientation.z = q_tf.getZ();
	marker->pose.orientation.w = q_tf.getW();
	marker->scale.x = 3;
	marker->scale.y = 1;
	marker->scale.z = 1.5;
	marker->color.a = 1.0; // Don't forget to set the alpha!
	marker->color.r = red;
	marker->color.g = green;
	marker->color.b = blue;
}
void visualization_text(visualization_msgs::Marker::Ptr marker,float x, float y,float velocity,float red, float green, float blue){
	//visualization_msgs::Marker marker_goal;
	marker->header.frame_id = "odom";
	marker->header.stamp = ros::Time();
	marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker->action = visualization_msgs::Marker::ADD;
	marker->pose.position.x = x+20;
	marker->pose.position.y = y-20;
	marker->pose.position.z = 1;
	std::ostringstream ss;
	ss << velocity << " kmph";
	//std::string s;
	marker->text = ss.str();
	marker->scale.x = 0.2;
	marker->scale.y = 0.2;
	marker->scale.z = 2;
	marker->color.a = 1.0; // Don't forget to set the alpha!
	marker->color.r = red;
	marker->color.g = green;
	marker->color.b = blue;
}

// ---------------- Callbacks ---------------------------------------------------------------
void callback(const nav_msgs::OdometryConstPtr& pos)
{		//int init_flag = 0;
		current_pts.x = pos->pose.pose.position.x;
		current_pts.y = pos->pose.pose.position.y;
   		if(init_flag == 0){
        	init_pos.x = pos->pose.pose.position.x;
        	init_pos.y = pos->pose.pose.position.y;
        	init_pos.heading = 0;
        	init_flag++;
        	std::cout<< "Initialized"<< std::endl;
        }
        
        
        else{
        	current_states.x = pos->pose.pose.position.x - init_pos.x;
        	current_states.y = pos->pose.pose.position.y - init_pos.y; 
        	// odom_pos.header = pos->header;
        	// odom_pos.pose.pose.position.x = current_positions.x;
        	// odom_pos.pose.pose.position.y = current_positions.y;
        	// odom_pos.pose.pose.position.z = 0;
        }

}

void speed_callback(const std_msgs::Float64::ConstPtr& msg){
	current_states.velocity = msg->data;
}
void compass_callback(const std_msgs::Float32::ConstPtr& msg){
	current_states.heading = msg->data * PI / 180;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "visual");
  	ros::NodeHandle node;

	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
	ros::Publisher vis_pub2 = node.advertise<visualization_msgs::Marker>( "visualization_marker_text", 1 );
	ros::Subscriber odom = node.subscribe("odom", 1, callback);
  	//ros::Subscriber compass = node.subscribe("compass", 1, compass_callback);
  	ros::Subscriber speed = node.subscribe("speed_signal", 1, speed_callback);

  	visualization_msgs::Marker::Ptr current_marker(new visualization_msgs::Marker);
	visualization_msgs::Marker::Ptr target_marker(new visualization_msgs::Marker);
	visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
	  
	tf::TransformBroadcaster br;
	tf::Transform transform;
	std::ofstream data_file; 
	data_file.open ("waypoints_data.txt");/// ------ open file for waypoints
	ros::Rate loop_rate(100);
	current_states.heading = 0;  /// ---- 0 heading
	//ros::Time begin = ros::Time::now().toSec();
	while(ros::ok()){
		transform.setOrigin( tf::Vector3(current_states.x, current_states.y, 0) );
		transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		ros::spinOnce(); 
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "velodyne"));
		tf::Quaternion q_tf = tf::Quaternion(0, 0, current_states.heading);
		visualization_text(marker,current_states.x, current_states.y,current_states.velocity,0,1,0);	
		vis_pub2.publish(*marker);
		visualization(current_marker, current_states.x, current_states.y,q_tf, 0.5,0.5,0.5);
		vis_pub.publish(*current_marker);
		int current_time = ros::Time::now().toSec();
		int check = current_time % 5;
		//std::cout<< current_states.velocity<< std::endl;
		if(current_states.velocity > 0 && check == 0)
		{
			if (data_file.is_open())
			  {
			    data_file << current_pts.x<<" "<< current_pts.y <<"\n";
			  }
			else std::cout << "Unable to open file";
		}

		loop_rate.sleep();
	}
	data_file.close();	
}
