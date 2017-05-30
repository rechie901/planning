#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <ecl/geometry.hpp>

using ecl::SmoothLinearSpline;
using ecl::CubicSpline;
using ecl::TensionSpline;

using namespace Eigen;
float wheel_radius = 0.079259/2;
float d=0.0944; //-- 2*d=distance between left and right wheels
float l=0.3222; //-- l=distance between front and rear wheels

float max_speed = 10.0; // m/s
float max_accel = 10.0;
float max_brake = 1.0; // m/(s*s)
float max_steervel = 1.0;
float max_steerangle = 40*3.14/180;
const int end_time = 200; 
int flag =0;
int steeringLeftHandle, steeringRightHandle,motorLeftHandle,motorRightHandle, targethandle, GPS ;  

struct state{
	float x[end_time],y[end_time],heading[end_time],steer_angle[end_time],velocity[end_time];
};
struct dynamics{
	float x,y,heading,steer_angle,velocity;
};
struct control{
	float acceleration[end_time],steer_vel[end_time];
};
struct control_parameters{
	float a1, b1, c1, a2, b2, c2;
};
struct terminal_states{
	float x, y, heading, steer_angle, velocity;
};

state compute_states(state v, control controller, float step_size, const int tg, float curr_x, float curr_y, float curr_heading, float curr_steerangle,float curr_velocity){
	float init_x = curr_x;
	float init_y = curr_y;
	float init_heading = curr_heading;
	float init_steerangle = curr_steerangle;
	float init_velocity = curr_velocity; 
	for (int j = 0; j < end_time; ++j)
	{
		v.velocity[j] = init_velocity + controller.acceleration[j]*step_size;
		if(v.velocity[j] > max_speed){
			v.velocity[j] = max_speed;
		}
		if(v.velocity[j] < 0){
			v.velocity[j] = 0;
		}
		 //std::cout << " predicted v "<< v.velocity[j] << std::endl;  
		init_velocity = v.velocity[j]; 
		
	}
	for (int i = 0; i < end_time; ++i)
	{
		v.steer_angle[i] = init_steerangle + controller.steer_vel[i]*step_size;
		if(v.steer_angle[i] > max_steerangle){
			v.steer_angle[i] = max_steerangle;
		}
		if (v.steer_angle[i] < -max_steerangle)
		{
			v.steer_angle[i] = -max_steerangle;
		}
		init_steerangle = v.steer_angle[i];
	}
	for (int i = 0; i < end_time; ++i)
	{
		v.heading[i] = init_heading + v.velocity[i] * tan(v.steer_angle[i])*step_size/l;
		init_heading = v.heading[i];
		v.x[i] = init_x + v.velocity[i] * cos(v.heading[i]) * step_size;
		init_x = v.x[i];
		//std::cout << " predicted x end "<< v.x[i] << std::endl;
		v.y[i] = init_y + v.velocity[i] * sin(v.heading[i]) * step_size;
		init_y = v.y[i];
	}
	// std::cout << " predicted x end "<< v.x[end_time-1] << std::endl;
	// std::cout << " predicted y end "<< v.y[end_time-1] << std::endl;
	// std::cout << " heading " << v.heading[end_time-1]*180/3.14 << std::endl;
	return(v);

}
control compute_u(control controller, control_parameters params, const int tg, float step_size){
	for (int i = 0; i < end_time; ++i)
	{
		 
		controller.acceleration[i] = params.c2 + 2*params.b2*step_size*i + 3*params.a2*pow(step_size*i ,2) ;
		controller.steer_vel[i] = params.c1 + 2*params.b1*step_size*i + 3*params.a1*pow(step_size*i ,2) ;
		if(controller.acceleration[i] > max_accel){
			controller.acceleration[i] = max_accel;
		}
		if(controller.acceleration[i] < -max_accel){
			controller.acceleration[i] = -max_accel;
		}
		if(controller.steer_vel[i] < -max_steervel){
			controller.steer_vel[i] = -max_steervel;
		}
		if(controller.steer_vel[i] >  max_steervel){
			controller.steer_vel[i] = max_steervel;
		}
		// if(controller.acceleration[i] > -1 && controller.acceleration[i]<1)
		 	//std::cout << " controller.acceleration " << controller.acceleration[i] << " i "<<i << std::endl;
	}
		
		return(controller);

}
float compute_j(terminal_states t_state, state final, float j, float beta[4], const int tg)
{
	j = sqrt(beta[0] * (final.x[end_time-1] - t_state.x)*(final.x[end_time-1] - t_state.x) + beta[1]*(final.y[end_time-1] - t_state.y)*(final.y[end_time-1] - t_state.y) + beta[2]*(final.velocity[end_time-1] - t_state.velocity)*(final.velocity[end_time-1] - t_state.velocity) + beta[3]*(final.heading[end_time-1] - t_state.heading)*(final.heading[end_time-1] - t_state.heading));
	//std::cout << " J " << j<< std::endl;
	return(j); 
}

control_parameters compute_cor(float e, control_parameters params, control controller, const int tg, float step_size, state current_state, terminal_states desired_state, float coeff, float curr_x, float curr_y, float curr_heading, float curr_steerpos, float curr_velocity)
{	
	control_parameters del_params;
	control_parameters new_params;
	state updated_state;
	VectorXf del_p(6);
	VectorXf del_s(5);
	MatrixXf jacobian_mat(5,6);

	for (int k = 0; k < 6; ++k)
	{
		new_params = params ; 
		if(k == 0)
			new_params.a1 = params.a1 + e;
		if (k == 1)
			new_params.b1 = params.b1 + e;
		if(k == 2)
			new_params.c1 = params.c1 + e;
		if (k == 3)
			new_params.a2 = params.a2 + e;
		if(k == 4)
			new_params.b2 = params.b2 + e;
		if (k == 5)
			new_params.c2 = params.c2 + e;
		//std::cout<< " params.a2 " << params.a2 << " new_params.a2 " << new_params.a2<< std::endl;
		controller = compute_u(controller, new_params, tg, step_size);
		updated_state = compute_states(updated_state, controller, step_size, tg, curr_x, curr_y, curr_heading, curr_steerpos, curr_velocity);
		
		
		jacobian_mat(0,k) = (current_state.x[end_time-1] - updated_state.x[end_time-1])/e ;
		//std::cout<< " current_state " << current_state.x[end_time-1]<< " updated_state " << updated_state.x[end_time-1]<< std::endl;

		jacobian_mat(1,k) = (current_state.y[end_time-1] - updated_state.y[end_time-1])/e ;
		jacobian_mat(2,k) = (current_state.heading[end_time-1] - updated_state.heading[end_time-1])/e ;
		jacobian_mat(3,k) = (current_state.steer_angle[end_time-1] - updated_state.steer_angle[end_time-1])/e ;
		jacobian_mat(4,k) = (current_state.velocity[end_time-1] - updated_state.velocity[end_time-1])/e ;

	}
	del_p = jacobian_mat.jacobiSvd(ComputeThinU | ComputeThinV).solve(del_s);
	params.a1 = -coeff*del_p[0] + params.a1;
	params.b1 = -coeff*del_p[1] + params.b1;
	params.c1 = -coeff*del_p[2] + params.c1;
	params.a2 = -coeff*del_p[3] + params.a2;
	params.b2 = -coeff*del_p[4] + params.b2;
	params.c2 = -coeff*del_p[5] + params.c2;
	//std::cout<< " param a2" << params.a2<< std::endl;
	return(params);
}
// void accelerate(int ID ,float accel_input,float angular_vel, float current_vel, float delta_t){
// 	if(current_vel < max_speed){
//         angular_vel = angular_vel + ((accel_input*delta_t)/wheel_radius); 
//     	//std::cout <<"angular_vel " << angular_vel << std::endl;
//     }
//     else
//         angular_vel = angular_vel;
    
//     simxSetJointTargetVelocity(ID,motorLeftHandle,angular_vel,simx_opmode_oneshot);            
//     simxSetJointTargetVelocity(ID,motorRightHandle,angular_vel,simx_opmode_oneshot);
//     //return(angular_vel);         	

// }
// void brake(int ID ,float brake_input,float angular_vel, float current_vel, float delta_t){
// 	   if(angular_vel > 0)
//         {			
//            	angular_vel = angular_vel - ((brake_input*delta_t)/wheel_radius);
//            	simxSetJointTargetVelocity(ID,motorLeftHandle,angular_vel,simx_opmode_oneshot);            
//            	simxSetJointTargetVelocity(ID,motorRightHandle,angular_vel,simx_opmode_oneshot);           
//  		}
//  		else
//  		{
//  			simxSetJointTargetVelocity(ID,motorLeftHandle,0,simx_opmode_oneshot);            
//         	simxSetJointTargetVelocity(ID,motorRightHandle,0,simx_opmode_oneshot);
//  			//std::cout << "brake "<< curr_vel<< std::endl;
//  			//simxStopSimulation(ID, simx_opmode_oneshot);
//  		}             	
//  		//return(angular_vel);
// } 
// void steer(int ID, float desiredSteeringAngle){
	
// 	float theta;
// 	if(desiredSteeringAngle < 0)
// 		theta = (90 + desiredSteeringAngle);
// 	else if(desiredSteeringAngle > 0)
// 		theta = -1 * (90 - desiredSteeringAngle);
// 	if(theta < - max_steerangle)
// 		theta = max_steerangle;
// 	if(theta > max_steerangle)
// 		theta = -1 * max_steerangle;
// 	theta = theta * 3.14/180;
// 	float steeringAngleLeft = atan(l/(-d+l/tan(theta)));
//     float steeringAngleRight = atan(l/(d+l/tan(theta)));

// }

int main(int argc,char* argv[])
{
    if (clientID!=-1)
    {
        //printf("Connected to remote API server\n");
        std::cout<< "Connected"<< std::endl;
        
        //auto time_start = get_time::now();
        float curr_pos[7], targetpos[5];  // x,y,z,theta,vel 
        targetpos[4] = 0;
        float accel = 199*2.5, deccel = 50;

        float ang_vel = 0.0;  // angular velocity of wheel joint
        float curr_vel = 0.0;
        float curr_steerpos = 0.0;
   
        //extApi_sleepMs(100);
        // float n = 0.0; // time
        // float step = 0.002; // step size -- 2 ms
        while ()
        {
        	
            float r = sqrt(targetpos[2]*targetpos[2] + targetpos[1]*targetpos[1]);
            float alpha = atan(targetpos[2]/targetpos[1]);
            curr_vel = sqrt(curr_pos[5]*curr_pos[5] + curr_pos[4]*curr_pos[4]); // current velocity of the vehicle
            alpha = alpha*180/3.14 ;

            const int t_g = end_time;
            float step_size = 0.2;
            float J ;
            float epsilon = 0.1;
            int counter = 0;
            float beta[4] = {0.01, 0.01, 0.01, 0.01};
            float tuning_coeff = 1;
            float e = 0.2;

            control_parameters params;
            params.a2 = -1.0;
            params.b2 = 2.0;
            params.c2 = 1.0;
            params.a1 = 1.0;
            params.b1 = 2.0;
            params.c1 = 1.0;
            control control_signals;

            state predicted_state;

            float curr_heading = atan(curr_pos[2]/curr_pos[1]);
         	terminal_states des_state;
            des_state.x = targetpos[0];
            des_state.y = targetpos[1];
            des_state.heading = atan(targetpos[0]/targetpos[1]);
            des_state.steer_angle = 0.0 ; // steer zero
            des_state.velocity = 0.0 ; // end vel zero

            std::cout<< " destination x " << des_state.x << " destination y " << des_state.y << " End Time " << end_time << std::endl;
            std::cout<< " current x " << curr_pos[1] << " current y " << curr_pos[2] <<" curr heading " << des_state.heading*180/3.14  <<  std::endl;
         

            do
            {	
            	control_signals = compute_u(control_signals, params, t_g, step_size);
				predicted_state =  compute_states(predicted_state,control_signals, step_size, t_g, curr_pos[1], curr_pos[2], curr_heading, curr_steerpos,curr_vel);
				J = compute_j(des_state, predicted_state, J, beta, t_g);
				params = compute_cor(e, params, control_signals, t_g, step_size, predicted_state, des_state, tuning_coeff, curr_pos[1], curr_pos[2], curr_heading, curr_steerpos, curr_vel);
				counter++;

            }while(counter < 20);
            
            std::cout << "J " << J << std::endl;
            std::cout << "params - a2 " << params.a2  << std::endl;
            std::cout << "params - b2 " << params.b2  << std::endl;
            std::cout << "params - c2 " << params.c2  << std::endl;
            std::cout << "params - a1 " << params.a1  << std::endl;
            std::cout << "params - b1 " << params.b1  << std::endl;
            std::cout << "params - c1 " << params.c1  << std::endl;
            std::cout<< " destination x " << des_state.x << " destination y " << des_state.y << " End Time " << end_time << std::endl;
            std::cout<< " current x " << curr_pos[1] << " current y " << curr_pos[2] <<" curr heading " << des_state.heading*180/3.14  <<  std::endl;
            std::cout << "final state x " << predicted_state.x[end_time-1]<<" y  "<<predicted_state.y[end_time-1]<<" velocity  "<<predicted_state.velocity[end_time-1]<<" heading  "<<predicted_state.heading[end_time-1]*3.14/180<< " steer pos  "<<predicted_state.steer_angle[end_time-1]*3.14/180<< std::endl;
            // std::cout << " control_signals " << control_signals.acceleration[0] << " , "<< control_signals.steer_vel[0] << std::endl;

        }


    }
    return(0);
}
