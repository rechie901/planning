#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <fstream>

//#include <Quaternion.h>

struct current_pos
 {
 	float x;
 	float y;
 	float theta;
 	float velocity;
 }; 

struct goal_pos
 {
 	float x;
 	float y;
 	float theta;
 	float velocity;
 };
struct controller{
	float throttle_val;
	float brake_val;
	float steer_val;
	float gear_val;
};


int init_flag = 0;
int target_flag = 0;
float max_speed = 10;                    // kmph
float max_throttle = 20;                 // %
float max_brake = 100;                   // %
float max_steerangle = 40;               // Degrees
float n = 0.0; // time
float step = 0.1; // step size -- 100 ms
float throttle_jerk = 2;
float brake_jerk = 1;
float steer_vel = 5;
std::vector<goal_pos> viz_input;

nav_msgs::Odometry odom_pos;
current_pos init_pos;
current_pos current_positions;
goal_pos target;

controller control;

float accelerate(float jerk ,float throttle, float current_vel, float delta_t){
	//std::cout << "throttle "<< throttle << std::endl;
	if(current_vel < max_speed){
		throttle = throttle + delta_t * jerk; 
    }

    else
        throttle = throttle;
    if(throttle > max_throttle)
    	throttle = max_throttle;
    return(throttle);         	
}

float brake(float jerk ,float brake_val, float current_vel, float delta_t){
	   if(current_vel > 0)
        {			
           	brake_val = brake_val + jerk * delta_t;           
 		}
 		else
 		{
 			brake_val = 100;
 		}
 		if(brake_val > max_brake)
 			brake_val = max_brake;

 		return(brake_val);
}
float steer(float steer_velocity, float steer_pos, float desired_theta, float current_theta, float delta_t){
	if(current_theta < 0)
		current_theta = 180 + current_theta;
	if(desired_theta < 0)
		desired_theta = 180 + desired_theta;
	float del = desired_theta - current_theta;
	
	if(del > 5){
		steer_pos = steer_pos - steer_velocity * delta_t;
	}
	else if(del < -5){
		steer_pos = steer_pos + steer_velocity * delta_t;
	}
	else
		steer_pos = 0;
	if(steer_pos < -40)
		steer_pos = -40;
	if(steer_pos > max_steerangle)
		steer_pos = max_steerangle;
	std::cout<< "delta: "<<del << " desired_theta: "<< desired_theta<<std::endl;
	return(steer_pos);
}

void callback(const nav_msgs::OdometryConstPtr& pos)
{
    
        
        if(init_flag == 0){
        	init_pos.x = pos->pose.pose.position.x;
        	init_pos.y = pos->pose.pose.position.y;
        	init_pos.theta = 0;
        	init_flag++;
        	std::cout<< "Initialized"<< std::endl;
        }
        
        
        else{
        	current_positions.x = pos->pose.pose.position.x - init_pos.x;
        	current_positions.y = pos->pose.pose.position.y - init_pos.y; 
        	odom_pos.header = pos->header;
        	odom_pos.pose.pose.position.x = current_positions.x;
        	odom_pos.pose.pose.position.y = current_positions.y;
        	odom_pos.pose.pose.position.z = 0;
        }

}
void speed_callback(const std_msgs::Float64::ConstPtr& msg){
	current_positions.velocity = msg->data;
}
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	//std::cout<< " Select the Target " << std::endl;
	// goal_pos input;
	// input.x = msg->pose.position.x;
	// input.y = msg->pose.position.y;
	// input.theta = 0;
	// input.velocity = 0;
	target.x = msg->pose.position.x;
	target.y = msg->pose.position.y;
	target.theta = 0;
	target.velocity = 0;
	//viz_input.push_back(input);

	target_flag = 1;

}
void visualization_car(visualization_msgs::Marker::Ptr marker, float pos_x, float pos_y, tf::Quaternion q_tf){
  //visualization_msgs::Marker marker;
  marker->header.frame_id = "base_link";
  marker->header.stamp = ros::Time();
  //marker->ns = "/odom";
  //marker->id = 0;
  marker->action = visualization_msgs::Marker::ADD;
  marker->type = visualization_msgs::Marker::MESH_RESOURCE;
  marker->mesh_resource = "/home/sarthak/catkin_ws/src/model_publisher/.config/model/default.urdf";
  marker->mesh_use_embedded_materials = true;
  marker->pose.position.x = pos_x;
  marker->pose.position.y = pos_y;
  marker->pose.position.z = 0;
  // double roll = offset_roll * (M_PI / 180.0);
  // double yaw = offset_yaw * (M_PI / 180.0);
  // double pitch = offset_pitch * (M_PI / 180.0);
	marker->pose.orientation.x = q_tf.getX();
	marker->pose.orientation.y = q_tf.getY();
	marker->pose.orientation.z = q_tf.getZ();
	marker->pose.orientation.w = q_tf.getW();
  marker->color.r = 0.0;
  marker->color.g = 0.0;
  marker->color.b = 0.0;
  marker->color.a = 0.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->frame_locked = true;
}
void visualization(visualization_msgs::Marker::Ptr marker, float pos_x, float pos_y,tf::Quaternion q_tf, float red, float green, float blue){
	//visualization_msgs::Marker marker_goal;
	marker->header.frame_id = "odom";
	marker->header.stamp = ros::Time();
	marker->type = visualization_msgs::Marker::SPHERE;
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
	marker->scale.z = 3;
	marker->color.a = 1.0; // Don't forget to set the alpha!
	marker->color.r = red;
	marker->color.g = green;
	marker->color.b = blue;
}
void visualization_arrow(visualization_msgs::Marker::Ptr marker, float pos_x, float pos_y,float last_x, float last_y, float red, float green, float blue){
	//visualization_msgs::Marker marker_goal;
	marker->header.frame_id = "odom";
	marker->header.stamp = ros::Time();
	marker->type = visualization_msgs::Marker::ARROW;
	marker->action = visualization_msgs::Marker::ADD;
	marker->pose.position.x = pos_x;
	marker->pose.position.y = pos_y;
	marker->pose.position.z = 1;
	// marker->pose.orientation.x = q_tf.getX();
	// marker->pose.orientation.y = q_tf.getX();
	// marker->pose.orientation.z = q_tf.getX();
	// marker->pose.orientation.w = q_tf.getX();
	marker->points[0].x = last_x;
	marker->points[0].y = last_y;
	marker->points[1].x = pos_x;
	marker->points[1].y = pos_y;
	marker->scale.x = 5;
	marker->scale.y = 1;
	marker->scale.z = 1;
	marker->color.a = 1.0; // Don't forget to set the alpha!
	marker->color.r = red;
	marker->color.g = green;
	marker->color.b = blue;
}
int main (int argc, char **argv) {
  ros::init(argc, argv, "p2pnav_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  std::cout<< "Connected"<< std::endl;
  target.x = 0;

  control.brake_val = 0;
  control.throttle_val = 0;
  control.steer_val = 0;
  current_positions.velocity = 0;
  float last_posx = 0;
  float last_posy = 0;
  float heading = 0;
  
  std::string steer_dir;
  // priv_node.param<std::string>("frame_id", frame_id, "");
  // priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  // priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("final_odom", 1);  // -- final odom --- 
  ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
  ros::Publisher vis_target_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker_target", 1 );
  ros::Publisher vis_arrow_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker_arrow", 1 );
  ros::Subscriber goal = node.subscribe("move_base_simple/goal", 1, goal_callback);
  ros::Subscriber odom = node.subscribe("odom", 1, callback);
  //ros::Subscriber can_input = node.subscribe("can_input", 1, can_callback);
  ros::Subscriber speed = node.subscribe("speed_signal", 1, speed_callback);
  ros::Rate loop_rate(10); //0.01 Second
  
  //std::ofstream myfile;
  //myfile.open ("/home/sarthak/catkin_ws/src/CAN_interfacing/data/params.dat");
  visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
  visualization_msgs::Marker::Ptr marker2(new visualization_msgs::Marker);
  visualization_msgs::Marker::Ptr marker3(new visualization_msgs::Marker);
  while(ros::ok()){
  	ros::spinOnce();

  	if(current_positions.velocity != 0)
	    	heading = atan((current_positions.y - last_posy)/(current_positions.x - last_posx));
  	if(target_flag == 1){ /// ---- change flag for input
  		// current_positions.x = viz_input[2].x;
  		// current_positions.y = viz_input[2].y;
  		// current_positions.velocity = 2;
  		float g_x = target.x - current_positions.x ;
  		float g_y = target.y - current_positions.y ;
	    float r = sqrt(g_y * g_y + g_x * g_x);
	    float alpha = atan(g_y/g_x);
	    alpha = alpha*180/3.14 ;
	    current_positions.theta = heading*180/3.14 ;      
	    //std::cout<< " here "<< std::endl;
	        
		if(r > 6) 
	    {	
	    	control.throttle_val = accelerate(throttle_jerk, control.throttle_val, current_positions.velocity, step);
	    	control.steer_val = steer(steer_vel, control.steer_val, alpha, current_positions.theta, step);
	    	if(current_positions.velocity > 10 && control.steer_val > 30 || control.steer_val<-30) 
	    		control.throttle_val = 10;   		  
	    }
	    else
	    {	
	       	control.brake_val = brake(brake_jerk ,control.brake_val, current_positions.velocity, step);
	       	control.throttle_val = 0;
	      	control.steer_val = steer(steer_vel, control.steer_val, alpha, current_positions.theta, step);
	      	if(current_positions.velocity == 0 && control.brake_val == 100)
	      		target_flag = 0;   ////             -------- Reset Flag after achieving Target-------
	    }
	    loop_rate.sleep();
	    n = n+step;
	    
	tf::Quaternion q_tf = tf::Quaternion(0, 0, heading);
	    odom_pos.pose.pose.orientation.x = q_tf.getX();
		odom_pos.pose.pose.orientation.y = q_tf.getY();
		odom_pos.pose.pose.orientation.z = q_tf.getZ();
		odom_pos.pose.pose.orientation.w = q_tf.getW();
	//visualization(marker, current_positions.x, current_positions.y,q_tf,0,1,0);


	 //    visualization_arrow(marker3, current_positions.x, current_positions.y, last_posx, last_posy, 0,1,0);
		// vis_arrow_pub.publish(*marker3);
	    visualization(marker2, target.x, target.y,q_tf, 1,0,0);
	    std::cout<< " heading : " << heading * 180/3.14 <<std::endl;
	    odom_pub.publish(odom_pos);
	    vis_target_pub.publish(*marker2);
	    
	    std::stringstream ss;
	    
	    if(control.steer_val <= 0)
	    	steer_dir = "RIGHT";
	    else
	    	steer_dir = "LEFT";
	    float steer_output = abs(control.steer_val);
	    
	    int throttle_output = int(control.throttle_val);
	    int brake_output = int(control.brake_val);
	    int steering_output = int(steer_output);
	    ss  << "/home/sarthak/catkin_ws/src/CAN_interfacing/scripts/sendCAN.sh "<< throttle_output<<" "<<brake_output<<" "<<steering_output<<" "<<steer_dir<<" FORWARD NONE OFF OFF OFF OFF";
		std::cout<<" n : "<< n << " R : " << r<< " throttle : "<< throttle_output << " brake_val : "<< brake_output << " steer_val : "<< control.steer_val << std::endl;

		//myfile  << control.throttle_val<<" "<<control.brake_val<<" "<<steer_output<<" "<<steer_dir<<" FORWARD NONE OFF OFF OFF OFF"<<std::endl;
	   	
	    system(ss.str().c_str());
	    ss.clear();
	}
	else
		std::cout<< "Select Target"<<std::endl;
	tf::Quaternion q_tf = tf::Quaternion(0, 0, heading);
	visualization(marker, current_positions.x, current_positions.y,q_tf,0,1,0);
	vis_pub.publish(*marker);



	// if(target_flag == 1){
	// 	target.x = viz_input[0].x;
	// 	target.y = viz_input[0].y;
	// }
	// if(target_flag == 2){
	// 	last_posx = viz_input[1].x;
	// 	last_posy = viz_input[1].y;
	// }
	
	last_posx = current_positions.x;
	last_posy = current_positions.y;
    
    // std::cout<< " current_positions.x : "<< current_positions.x << " y : "<< current_positions.y << " velocity : "<< current_positions.velocity << std::endl;
    // std::cout<< " goal pos : x "<< target.x << " goal_pos y "<< target.y << std::endl;

  }

  
}