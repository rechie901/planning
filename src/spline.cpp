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
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>
#include "spline.h"


#define max_accel 100   //percent throttle
#define PI 3.14159
#define max_speed 8   //kmph
#define l 2.1          // wheelbase length in meters

int max_throttle = 40;
float len_factor = 1.6;
float target_flag = 0;
float init_flag = 0;
float max_steerangle = 40 * PI/180; // max steer angle in rad

struct state{
	float x, y, heading, steer_angle, velocity;
};
struct control{
	float acceleration,kx,ky,steer_angle;
};
struct curve
{
	float kx, ky;
};
struct control_parameters{
	float a1, b1, c1, k1_x, k1_y, k2_x, k2_y,t_a,t_b,v_0;
};
struct can_output{
	float throttle, brake, steer_angle, steer_dir;
};
struct waypoints{
	double x, y;
};

state current_states;
state terminal_states;
state mid_states;
state init_pos;

float cubic_spline(std::vector<waypoints> &points,std::vector<curve> &curve_pts,state current_pos){
	float length = 0;
	tk::spline sx,sy;
	std::vector<double> t,x,y;
	t.resize(3);
	t[0] = 0 ; t[1] = 15; t[2] = 29;
	for (int i = 0; i < points.size(); ++i)
	{
		x.push_back(points[i].x);
		y.push_back(points[i].y);
	}
   	sx.set_points(t,x);
   	sy.set_points(t,y);
   	curve_pts.resize(points.size()*10);
   	std::cout<<curve_pts.size()<<std::endl;
   	for (int i = 0; i < curve_pts.size(); ++i)
   	{
		curve_pts[i].kx = sx(i);
		curve_pts[i].ky = sy(i);
		if(i>0){
			length = length + sqrt((curve_pts[i].kx-curve_pts[i-1].kx)*(curve_pts[i].kx- curve_pts[i-1].kx) + (curve_pts[i].ky-curve_pts[i-1].ky)*(curve_pts[i].ky-curve_pts[i-1].ky));
			//theta = theta + abs(atan((path[i].ky - path[i-1].ky)/path[i].kx - path[i-1].kx));
		}
		std::cout<<i <<" length "<< length<<" x "<< sx(i) << " y "<<curve_pts[i].ky  <<  std::endl;
   	}
   	
   	return length;
}

void compute_states(std::vector<state>& states, state current_states, std::vector<control>& controller, float step_size){
	float init_x = current_states.x;
	float init_y = current_states.y;
	float init_heading = current_states.heading;
	float init_velocity = current_states.velocity;
	float last_heading = init_heading;
	float del_theta=0;
	states.resize(controller.size()-1); 
	for (int j = 0; j < controller.size()-1; ++j)
	{
		states[j].velocity = init_velocity+ controller[j].acceleration*step_size;
		// if(states[j].velocity > max_speed*5/18){
		// 	states[j].velocity = max_speed*5/18;
		// }
		if(states[j].velocity < 0){
			states[j].velocity = 0;
		}
		//std::cout << " predicted v "<< states[j].velocity << std::endl;  
		init_velocity = states[j].velocity; 	
	}

	for (int it = 0; it < controller.size()-1; ++it)
	{	
		float des_heading = atan((controller[it+1].ky - controller[it].ky)/(controller[it+1].kx - controller[it].kx));
		if(des_heading < 0 && (controller[it+1].kx - controller[it].kx) < 0)
			des_heading = PI + des_heading;
		if(des_heading < 0 && (controller[it+1].kx - controller[it].kx) > 0)
			des_heading = des_heading;
		controller[it].steer_angle = atan((des_heading-last_heading)*l/(states[it].velocity*step_size));
		//del_theta = del_theta + fabs(atan(des_heading - last_heading));
		//del_theta = atan(des_heading - last_heading);
		
		if(controller[it].steer_angle > max_steerangle)
			controller[it].steer_angle = max_steerangle;
		if(controller[it].steer_angle < -max_steerangle)
			controller[it].steer_angle = -max_steerangle;
		//std::cout << " des_heading : "<< des_heading* 180/PI <<" diff : " <<(des_heading - last_heading) * 180/PI<<" last_heading " <<last_heading * 180/PI<< " steer_angle : "<<controller[it].steer_angle * 180/PI<< std::endl;
		last_heading = des_heading;
	}
	// /std::cout << " del_theta " << del_theta * 180/PI << std::endl;
	for (int i = 0; i < controller.size()-1; ++i)
	{
		states[i].heading = init_heading + states[i].velocity * tan(controller[i].steer_angle)*step_size/l;
		init_heading = states[i].heading;
		states[i].x = init_x + states[i].velocity * cos(states[i].heading) * step_size;
		init_x = states[i].x;
		//std::cout << " steer_angle : "<< states[i].heading * 180/PI << std::endl;
		states[i].y = init_y + states[i].velocity * sin(states[i].heading) * step_size;
		init_y = states[i].y;
	}
	// std::cout << " predicted x end "<< v.x[end_time-1] << std::endl;
	// std::cout << " predicted y end "<< v.y[end_time-1] << std::endl;
	// std::cout << " heading " << v.heading[end_time-1]*180/3.14 << std::endl;

}


float compute_curve(std::vector<curve> &path,control_parameters params,float k0_x, float k0_y,float step_size){
	float length = 0;
	float theta = 0;
	int n = 1/step_size;
	path.resize(n);
	for (int i = 0; i < n; ++i)
	{	
		float t = i * step_size;
		path[i].kx = (1 - t) * (1 - t) * k0_x + 2 * (1 - t) * t * params.k1_x+ t * t * params.k2_x;
		path[i].ky = (1 - t) * (1 - t) * k0_y + 2 * (1 - t) * t * params.k1_y+ t * t * params.k2_y;
		if(i>0){
			length = length + sqrt((path[i].kx-path[i-1].kx)*(path[i].kx-path[i-1].kx) + (path[i].ky-path[i-1].ky)*(path[i].ky-path[i-1].ky));
			//theta = theta + abs(tan((path[i].ky - path[i-1].ky)/path[i].kx - path[i-1].kx));
		}

		length = length;
	}
	std::cout << " length " << length << std::endl;
	//std::cout << " theta change " << theta << std::endl;
	return length;
}


void compute_u(std::vector<control> &controller,std::vector<curve> &curve_pts,control_parameters params,float k0_x, float k0_y,float length, float step_size, float init_velocity, float terminal_velocity, float last_heading){
	//int n = length / 2;
	//float tg = n*step_size;//len_factor * length/ params.v_0;
	int n = curve_pts.size();//tg/step_size;
	float tg = n*step_size;
	controller.resize(n);
	params.t_a = tg/3;
	params.t_b = 2*tg/3;
	float accel_0 = (params.v_0 - init_velocity)/params.t_a;
	float accel_f = (terminal_velocity - params.v_0)/(tg-params.t_b);
	float del_theta = 0;
	float inflection_point[2];
	for (int i = 0; i < n/3; ++i)
	{
			//if(current_velocity < v_0)
		controller[i].acceleration = accel_0;
			// else
			// 	controller[i].acceleration = 0;
	}
	for (int i = n/3; i < 2*n/3; ++i)
	{
		controller[i].acceleration = 0;
	}
	for (int i = 2*n/3; i < n; ++i)
	{
		controller[i].acceleration = accel_f;
	}
	for (int i = 0; i < curve_pts.size(); ++i)
   	{
		controller[i].kx = curve_pts[i].kx;
		controller[i].ky = curve_pts[i].ky;
		// if(i>0){
		// 	length = length + sqrt((curve_pts[i].kx-curve_pts[i-1].kx)*(curve_pts[i].kx- curve_pts[i-1].kx) + (curve_pts[i].ky-curve_pts[i-1].ky)*(curve_pts[i].ky-curve_pts[i-1].ky));
		// 	//theta = theta + abs(tan((path[i].ky - path[i-1].ky)/path[i].kx - path[i-1].kx));
		// }
		// std::cout<<i <<" length "<< length<<" x "<< curve_pts[i].kx << " y "<<curve_pts[i].ky  <<  std::endl;
   	}
	// for (int i = 0; i < n; ++i)
	// {
	// 	float t = i * 1/float(n);
	// 	controller[i].kx = (1 - t) * (1 - t) * k0_x + 2 * (1 - t) * t * params.k1_x+ t * t * params.k2_x;
	// 	controller[i].ky = (1 - t) * (1 - t) * k0_y + 2 * (1 - t) * t * params.k1_y+ t * t * params.k2_y;
	// 	if(i > 0){
	// 		float des_heading = atan((controller[i+1].ky - controller[i].ky)/(controller[i+1].kx - controller[i].kx));
	// 		del_theta = del_theta+fabs(atan(des_heading - last_heading));
	// 		if(fabs(atan(des_heading - last_heading)) > 0.8)
	// 		{
	// 			inflection_point[0] = i; ////
	// 		}
	// 		//std::cout<< " avg : " << del_theta * 180/PI << " delta : "<< fabs(atan(des_heading - last_heading))*180/PI << std::endl;
	// 		last_heading = des_heading; 
	// 	}
			
	// }
		
}
// void mapper(float accel_output, float steer_output, can_output output){
// 	if(accel_output >= 0 ){
// 		if(current_state.velocity < desired_velocity)
// 			output.throttle = output.throttle + 2;
// 		else
// 			output.throttle = output.throttle;
// 		if(output.throttle > 100)
// 			output.throttle = 100;
// 		if(output.throttle < 0)
// 			output.throttle = 0;
// 		output.brake = 0;
// 	}
// 	if (accel_output < 0)
// 	{
// 		if(current_velocity > desired_velocity)
// 			output.brake = output.brake + 2;
// 		else
// 			output.brake = output.brake;
// 		if(output.brake > 100)
// 			output.brake = 100;
// 		if(output.brake < 0)
// 			output.brake = 0;
// 		output.throttle = 0;
// 	}
// 	output.steer_angle = steer_output;
// 	if(steer_output > max_steerangle)
// 		output.steer_angle = max_steerangle;
// 	if (steer_output < -max_steerangle)
// 		output.steer_angle = -1*max_steerangle;


// }


void control_output(float &length, float target_dis,state current_state, state last_state,std::vector<control> &controller,std::vector<state>& states, control_parameters params,can_output &output, float step_size){
	length = length + sqrt((current_state.x - last_state.x) * (current_state.x - last_state.x) + (current_state.y - last_state.y) * (current_state.y - last_state.y));
	float tg = len_factor * length/ params.v_0;
	int n = tg/step_size;
	float accel_output = controller[n].acceleration;
	float steer_output = controller[n].steer_angle;
	//float del_d = sqrt((states[n].x - states[n-1].x) * (states[n].x - states[n-1].x) + (states[n].y - states[n-1].y) * (states[n].y - states[n-1].y)) - sqrt((current_state.x - last_state.x) * (current_state.x - last_state.x) + (current_state.y - last_state.y) * (current_state.y - last_state.y)); 
	float del_v = states[n].velocity - current_state.velocity;
	float del_theta = states[n].heading - current_state.heading;
	//accel_output = accel_output + del_v/tg;
	steer_output = steer_output + atan(del_theta * current_state.velocity/l)/tg;

	if(accel_output >= 0 ){
		output.brake = 0;
		if(current_state.velocity < (states[n].velocity + del_v))
			{output.throttle = output.throttle + 0.5;}
		else
			output.throttle = output.throttle;
		//std::cout <<" throttle before " <<output.throttle << std::endl;
		if(output.throttle > max_throttle)
			{output.throttle = max_throttle;}
		if(output.throttle < 0)
			output.throttle = 0;
		//std::cout <<" throttle after" <<output.throttle << std::endl;
	}
	if (accel_output < 0)
	{
		output.throttle = 0;
		if(current_state.velocity > states[n].velocity)
			output.brake = output.brake + 0.5;
		else
			output.brake = output.brake;
		if(output.brake > 100)
			output.brake = 100;
		if(output.brake < 0)
			output.brake = 0;
	}
	output.steer_angle = steer_output;
	if(steer_output > max_steerangle)
		output.steer_angle = max_steerangle;
	if (steer_output < -max_steerangle)
		output.steer_angle = -1*max_steerangle;
	if(steer_output >= 0)
		output.steer_dir = 1;
	if(steer_output < 0){
		output.steer_dir = 0;
		output.steer_angle = -1*output.steer_angle;
	}


}


//////-------------------------- Visualization Markers ----------------------------------

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
void visualization_text(visualization_msgs::Marker::Ptr marker,float velocity,float red, float green, float blue){
	//visualization_msgs::Marker marker_goal;
	marker->header.frame_id = "odom";
	marker->header.stamp = ros::Time();
	marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker->action = visualization_msgs::Marker::ADD;
	marker->pose.position.x = 50;
	marker->pose.position.y = 50;
	marker->pose.position.z = 1;
	std::ostringstream ss;
	ss << velocity*18/5 << " kmph";
	//std::string s;
	marker->text = ss.str();
	marker->scale.x = 1;
	marker->scale.y = 1;
	marker->scale.z = 10;
	marker->color.a = 1.0; // Don't forget to set the alpha!
	marker->color.r = red;
	marker->color.g = green;
	marker->color.b = blue;
}

//// ---------------- Callbacks ---------------------------------------------------------------
void callback(const nav_msgs::OdometryConstPtr& pos)
{		//int init_flag = 0;
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
// void compass_callback(const std_msgs::Float32::ConstPtr& msg){
// 	current_states.heading = msg->data * PI / 180;
// }
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	//std::cout<< " Select the Target " << std::endl;
	// goal_pos input;
	// input.x = msg->pose.position.x;
	// input.y = msg->pose.position.y;
	// input.theta = 0;
	// input.velocity = 0;
	
	if(target_flag == 0){
		terminal_states.x = msg->pose.position.x;
		terminal_states.y = msg->pose.position.y;
		terminal_states.heading = 0;
		terminal_states.velocity = 0;
		
	}
	if(target_flag == 1){
		mid_states.x = msg->pose.position.x;
		mid_states.y = msg->pose.position.y;
	}
	target_flag = target_flag + 1;
	//viz_input.push_back(input);

	

}


int main(int argc, char **argv){
	ros::init(argc, argv, "spline_node");
  	ros::NodeHandle node;
	//CubicSpline cubic1, cubic2;
	//SmoothLinearSpline spline1, spline2;
	ros::Publisher pub = node.advertise<nav_msgs::Path>("path", 10);
	ros::Publisher pub2 = node.advertise<nav_msgs::Path>("expected_path", 10);
	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
	ros::Publisher target_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker_target", 10 );
	ros::Publisher vis_pub2 = node.advertise<visualization_msgs::Marker>( "visualization_marker_text", 10 );

	ros::Subscriber goal = node.subscribe("move_base_simple/goal", 1, goal_callback);
	ros::Subscriber odom = node.subscribe("odom", 5, callback);
  	//ros::Subscriber can_input = node.subscribe("can_input", 1, can_callback);
  	//ros::Subscriber compass = node.subscribe("compass", 1, compass_callback);
  	ros::Subscriber speed = node.subscribe("speed_signal", 1, speed_callback);

  	visualization_msgs::Marker::Ptr current_marker(new visualization_msgs::Marker);
	visualization_msgs::Marker::Ptr target_marker(new visualization_msgs::Marker);
	visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
	
	nav_msgs::Path path;
	path.header.frame_id = "odom";
	path.header.stamp = ros::Time();
	
	nav_msgs::Path expected;
	expected.header.frame_id = "odom";
	expected.header.stamp = ros::Time();

	std::vector<control> controller;
	std::vector<state> states;
	std::vector<curve> curve_vec;
	std::vector<waypoints> way_points; 


	control_parameters params;
	can_output c_output;
	c_output.throttle = 0;
	c_output.brake = 0;
	c_output.steer_angle = 0;
	c_output.steer_dir= 0;
	float length;
	float current_length = 0;
	float step_size = 0.1;
	way_points.resize(3);

	//int tg = 1;
	// state current_states;
	// current_states.x = 0;
	// current_states.y = 0;
	// current_states.velocity = 0;
	current_states.heading = 0*PI/180;
	
	// state terminal_states;
	// terminal_states.x = 25;
	// terminal_states.y = 35;
	// terminal_states.velocity = 0;

	state last_state;
	last_state.x = 0;
	last_state.x = 0;

	int print_switch =0;
	ros::Rate loop_rate(100);
	std::cout << "Connected "<< std::endl;
	while(ros::ok()){
		ros::spinOnce();
		if(target_flag == 0 || target_flag == 1){
			if(print_switch == 0)
				std::cout<< "Please select the target and mid point " << std::endl;
			print_switch = 1;
		}
		if(current_states.velocity != 0){
			current_states.heading = atan((current_states.y - last_state.y)/(current_states.x - last_state.x));
			if(current_states.heading < 0 && (current_states.x - last_state.x) < 0)
				current_states.heading = PI + current_states.heading;
			if(current_states.heading < 0 && (current_states.x - last_state.x) > 0)
				current_states.heading = current_states.heading;
		}
		if (target_flag == 2)
		{

			params.a1 = 0;
			params.b1 = 0;
			params.c1 = 1; 
			params.k1_x = mid_states.x;//41.923;//
			params.k1_y = mid_states.y; //-6.422;//
			params.k2_x = terminal_states.x;//41.604;//
			params.k2_y = terminal_states.y;-36.988;//
			way_points[0].x = 0;
			way_points[0].y = 0;
 			way_points[1].x = mid_states.x;
			way_points[1].y = mid_states.y;
			way_points[2].x = terminal_states.x;
			way_points[2].y = terminal_states.y;
			params.v_0 = max_speed * 5/18;
			float end_pointerror = 0;

			length = cubic_spline(way_points,curve_vec,current_states); //compute_curve(curve_vec,params,current_states.x,current_states.y,0.01);
			expected.poses.resize(curve_vec.size());

			compute_u(controller,curve_vec,params,current_states.x,current_states.y,length,step_size,current_states.velocity,terminal_states.velocity,current_states.heading);
			compute_states(states,current_states,controller,step_size);
			end_pointerror = sqrt(pow((params.k2_x - states[states.size()-1].x),2)+ pow((params.k2_y - states[states.size()-1].y),2));
			std::cout << " error " << end_pointerror<< std::endl;
			// if(end_pointerror > 1){
			// 	for(int i=0; i<3 ; i++ ){
			// 		params.v_0 = params.v_0 - end_pointerror/(len_factor * length/ params.v_0); 
			// 		compute_u(controller,params,current_states.x,current_states.y,length,step_size, current_states.velocity, terminal_states.velocity);
			// 		compute_states(states,current_states,controller,step_size);
			// 		std::cout << " param v0 " << params.v_0<< std::endl;
			// 		std::cout << " sizes " << controller.size()<< " state " << states.size()<< std::endl;
			// 	}
			// } 
			// path.poses.resize(states.size());
			// for(int i ; i < states.size(); i++){
			// 		path.poses[i].pose.position.x = states[i].x;
			// 		path.poses[i].pose.position.y = states[i].y;
			// }
			for(int j ; j < curve_vec.size(); j++){
				expected.poses[j].pose.position.x = curve_vec[j].kx;
				expected.poses[j].pose.position.y = curve_vec[j].ky;
			}			
			//std::cout <<" states size " <<states.size()<<" curve size " <<curve_vec.size()<< std::endl;
			target_flag = 3;

		}
		//std::cout << current_states.heading * 180/PI<< std::endl;
		tf::Quaternion q_tf = tf::Quaternion(0, 0, current_states.heading);
		if(target_flag > 2){
			
			visualization(target_marker, terminal_states.x, terminal_states.y,q_tf,1,0,0); // change terminal 
			visualization_text(marker,current_states.velocity,0,1,0);
			//current_length = 50; // 
			if(length < current_length){
			
				c_output.throttle = 0;
				std:: cout << "target dis " << length << " travelled "<< current_length<< " output : "<< c_output.brake <<std::endl;
				c_output.brake = c_output.brake + 5;
				if(c_output.brake > 100)
					c_output.brake = 100;
				std:: cout << "target dis " << length << " travelled "<< current_length<< " output : "<< c_output.brake <<std::endl;
			}
			else
				control_output(current_length,length, current_states, last_state, controller, states, params, c_output,step_size);	
			
			//std::cout <<" throttle " <<c_output.throttle <<" brake " << c_output.brake <<" angle " << c_output.steer_angle << " dir " <<c_output.steer_dir<< std::endl;
			std::string steer_dir;
			if(c_output.steer_dir <= 0)
		    	steer_dir = "RIGHT";
		    else
		    	steer_dir = "LEFT";
		    std::stringstream ss;
		    int throttle_output = int(c_output.throttle);
		    int brake_output = int(c_output.brake);
		    int steering_output = int(c_output.steer_angle);
		    // ss  << "/home/sarthak/catkin_ws/src/CAN_interfacing/scripts/sendCAN.sh "<< throttle_output<<" "<<brake_output<<" "<<steering_output<<" "<<steer_dir<<" FORWARD NONE OFF OFF OFF OFF";		   	
		    // system(ss.str().c_str());
		    // ss.clear();
			last_state = current_states;
			//pub.publish(path);
			pub2.publish(expected);
			target_pub.publish(*target_marker);
			vis_pub2.publish(*marker);

		}
		visualization(current_marker, current_states.x, current_states.y,q_tf, 0,1,0);
		
		vis_pub.publish(*current_marker);
		loop_rate.sleep();
		}
	
	return 0;
}
