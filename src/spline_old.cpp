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


#define max_accel 100   //percent throttle
#define PI 3.14159
#define max_speed 20   //kmph
#define l 2.1          // wheelbase length in meters

// using ecl::SmoothLinearSpline;
// using ecl::CubicSpline;
// using ecl::TensionSpline;
// using ecl::Array;

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
	int throttle, brake, steer_angle, steer_dir;
};
void compute_states(std::vector<state>& states, state current_states, std::vector<control>& controller, float step_size){
	float init_x = current_states.x;
	float init_y = current_states.y;
	float init_heading = current_states.heading;
	float init_velocity = current_states.velocity;
	float last_heading = init_heading;
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
		
		if(controller[it].steer_angle > max_steerangle)
			controller[it].steer_angle = max_steerangle;
		if(controller[it].steer_angle < -max_steerangle)
			controller[it].steer_angle = -max_steerangle;
		//std::cout << " des_heading : "<< des_heading* 180/PI <<" diff : " <<(des_heading - last_heading) * 180/PI<<" last_heading " <<last_heading * 180/PI<< " steer_angle : "<<controller[it].steer_angle * 180/PI<< std::endl;
		last_heading = des_heading;
	}
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
	int n = 1/step_size;
	path.resize(n);
	for (int i = 0; i < n; ++i)
	{	
		float t = i * step_size;
		path[i].kx = (1 - t) * (1 - t) * k0_x + 2 * (1 - t) * t * params.k1_x+ t * t * params.k2_x;
		path[i].ky = (1 - t) * (1 - t) * k0_y + 2 * (1 - t) * t * params.k1_y+ t * t * params.k2_y;
		if(i>0)
			length = length + sqrt((path[i].kx-path[i-1].kx)*(path[i].kx-path[i-1].kx) + (path[i].ky-path[i-1].ky)*(path[i].ky-path[i-1].ky));
		length = length;
	}
	std::cout << " length " << length << std::endl;
	return length;
}
void compute_u(std::vector<control> &controller,control_parameters params,float k0_x, float k0_y,float length, float step_size, float init_velocity, float terminal_velocity){
	//int n = length / 2;
	float tg = 2 * length/ params.v_0;
	int n = tg/step_size;
	controller.resize(n);
	params.t_a = tg/3;
	params.t_b = 2*tg/3;
	float accel_0 = (params.v_0 - init_velocity)/params.t_a;
	float accel_f = (terminal_velocity - params.v_0)/(tg-params.t_b);
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
	for (int i = 0; i < n; ++i)
	{
		float t = i * 1/float(n);
		controller[i].kx = (1 - t) * (1 - t) * k0_x + 2 * (1 - t) * t * params.k1_x+ t * t * params.k2_x;
		controller[i].ky = (1 - t) * (1 - t) * k0_y + 2 * (1 - t) * t * params.k1_y+ t * t * params.k2_y;
		
	}
		
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
void control_output(float &length, state current_state, state last_state,std::vector<control> &controller,std::vector<state>& states, control_parameters params,can_output &output, float step_size){
	length = length + sqrt((current_state.x - last_state.x) * (current_state.x - last_state.x) + (current_state.y - last_state.y) * (current_state.y - last_state.y));
	float tg = 2 * length/ params.v_0;
	int n = tg/step_size;
	float accel_output = controller[n].acceleration;
	float steer_output = controller[n].steer_angle;
	//float del_d = sqrt((states[n].x - states[n-1].x) * (states[n].x - states[n-1].x) + (states[n].y - states[n-1].y) * (states[n].y - states[n-1].y)) - sqrt((current_state.x - last_state.x) * (current_state.x - last_state.x) + (current_state.y - last_state.y) * (current_state.y - last_state.y)); 
	float del_v = states[n].velocity - current_state.velocity;
	float del_theta = states[n].heading - current_state.heading;
	accel_output = accel_output + del_v/step_size;
	steer_output = steer_output + atan(del_theta * current_state.velocity/l);

	if(accel_output >= 0 ){
		if(current_state.velocity < states[n].velocity)
			output.throttle = output.throttle + 2;
		else
			output.throttle = output.throttle;
		if(output.throttle > 100)
			output.throttle = 100;
		if(output.throttle < 0)
			output.throttle = 0;
		output.brake = 0;
	}
	if (accel_output < 0)
	{
		if(current_state.velocity > states[n].velocity)
			output.brake = output.brake + 2;
		else
			output.brake = output.brake;
		if(output.brake > 100)
			output.brake = 100;
		if(output.brake < 0)
			output.brake = 0;
		output.throttle = 0;
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
int main(int argc, char **argv){
	ros::init(argc, argv, "spline_node");
  	ros::NodeHandle node;
	//CubicSpline cubic1, cubic2;
	//SmoothLinearSpline spline1, spline2;
	ros::Publisher pub = node.advertise<nav_msgs::Path>("path", 1);
	ros::Publisher pub2 = node.advertise<nav_msgs::Path>("expected_path", 1);
	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
	ros::Publisher vis_pub2 = node.advertise<visualization_msgs::Marker>( "visualization_marker_text", 1 );
	nav_msgs::Path path,expected;
	path.header.frame_id = "odom";
	path.header.stamp = ros::Time();
	expected.header.frame_id = "odom";
	expected.header.stamp = ros::Time();

	std::vector<control> controller;
	std::vector<state> states;
	std::vector<curve> curve_vec;
	can_output c_output;
	float length;
	float current_length = 0;
	float step_size = 0.1;
	//int tg = 1;
	state current_states;
	current_states.x = 0;
	current_states.y = 0;
	current_states.velocity = 0;
	current_states.heading = 69.2*PI/180;
	
	state terminal_states;
	terminal_states.x = 25;
	terminal_states.y = 35;
	terminal_states.velocity = 0;

	state last_state;
	last_state.x = 0;
	last_state.x = 0;

	control_parameters params;
	params.a1 = 0;
	params.b1 = 0;
	params.c1 = 1; 
	params.k1_x = 3;
	params.k1_y = 8; 
	params.k2_x = terminal_states.x;
	params.k2_y = terminal_states.y;
	params.v_0 = 5;

	length = compute_curve(curve_vec,params,current_states.x,current_states.y,0.01);
	compute_u(controller,params,current_states.x,current_states.y,length,step_size, current_states.velocity, terminal_states.velocity);
	compute_states(states,current_states,controller,step_size);
	path.poses.resize(states.size());
	expected.poses.resize(curve_vec.size());
	for(int i ; i < states.size(); i++){
		path.poses[i].pose.position.x = states[i].x;
		path.poses[i].pose.position.y = states[i].y;
	}
	for(int i ; i < curve_vec.size(); i++){
		expected.poses[i].pose.position.x = curve_vec[i].kx;
		expected.poses[i].pose.position.y = curve_vec[i].ky;
	}
	visualization_msgs::Marker::Ptr marker2(new visualization_msgs::Marker);
	visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
	ros::Rate loop_rate(100);
	while(ros::ok()){

		// for (int i = 0; i < states.size(); ++i)
		// {
		
		tf::Quaternion q_tf = tf::Quaternion(0, 0, current_states.heading);
		visualization(marker2, current_states.x, current_states.y,q_tf, 1,0,0);
		visualization_text(marker,current_states.velocity, 1,0,0);

		// }
		control_output(current_length, current_states, last_state, controller, states, params, c_output,step_size);
		std::cout <<" throttle " <<c_output.throttle <<" brake " << c_output.brake <<" angle " << c_output.steer_angle << " dir " <<c_output.steer_dir<< std::endl;
		last_state = current_states;
		vis_pub.publish(*marker2);
		vis_pub2.publish(*marker);
		pub.publish(path);
		pub2.publish(expected);
		loop_rate.sleep();
		ros::spinOnce();
		
	}
	
	return 0;
}
