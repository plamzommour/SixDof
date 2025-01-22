// This script handles the body-frame to local-level kinematics 

#include <cmath>
#include "./kinematics_driver.h"
#include "./vector.h"
#include "../dynamics/integrate.h"

void ll_kin(comvar* s_data)
{
	// Variable Definitions for Intermediate Integration Steps
	double q_body_2_ll_int [4]; 
	double q_ll_2_body_int [4]; 
	double q_body_2_ll_temp [4]; 
	double q_ll_2_body_temp [4]; 
	
	// Extract Data from s_data and Vectorize it //
	
	// Linear Accelerations 
	Vector acc_body; 
	Vector acc_ll;
	acc_body.x = s_data->u_dot_body; 
	acc_body.y = s_data->v_dot_body;
	acc_body.z = s_data->w_dot_body;  
	
	// Linear Velocities 
	Vector vel_body; 
	Vector vel_ll;
	vel_body.x = s_data->u_body; 
	vel_body.y = s_data->v_body;
	vel_body.z = s_data->w_body;  
	
	// Rotational Velocities 
	Vector rot_vel_body; 
	rot_vel_body.x = s_data->p_body; 
	rot_vel_body.y = s_data->q_body;
	rot_vel_body.z = s_data->r_body;  	
	
	// Linear Position
	Vector pos_body;
	Vector pos_ll;  
	pos_body.x = s_data->x_body; 
	pos_body.y = s_data->y_body;
	pos_body.z = s_data->z_body;  
	
	// Rotational Position 
	Vector rot_pos_body; 
	Vector rot_pos_ll;
	rot_pos_body.x = s_data->x_rot_body; 
	rot_pos_body.y = s_data->y_rot_body;
	rot_pos_body.z = s_data->z_rot_body;
	
	// Quaternion Derivative
	Quaternion q_body_2_ll_dot; 
	
	// Euler Angles 
	Vector eulers; 
	
	// Rotate EOM Outputs to Local Level from Body Frame and Save // 
	
	// Propogate Body to Local Level Quaternion and Integrate -- To-Do Move to First Kinematic Operation
	q_body_2_ll_dot = s_data->q_body_2_ll.q_dot(rot_vel_body); 
	q_body_2_ll_temp[0] = q_body_2_ll_dot.w; 
	q_body_2_ll_temp[1] = q_body_2_ll_dot.x; 
	q_body_2_ll_temp[2] = q_body_2_ll_dot.y; 
	q_body_2_ll_temp[3] = q_body_2_ll_dot.z;
	
		// Integrate Results for Body Frame Dynamics - Rates 
		for (int i=0; i<4; i++)
		{ 
			q_body_2_ll_int[i] += q_body_2_ll_temp[i] * s_data->dt; 
		
		}
	
	// Pack Integrated Result into Body to Local Level Quaternion
	s_data->q_body_2_ll.w = q_body_2_ll_int[0]; 
	s_data->q_body_2_ll.x = q_body_2_ll_int[1]; 
	s_data->q_body_2_ll.y = q_body_2_ll_int[2]; 
	s_data->q_body_2_ll.z = q_body_2_ll_int[3]; 
	
	// Normalize the New Quaternion 
	s_data->q_body_2_ll.normalize(); 
	
	// Find New Local Level to Body Quaternion 
	s_data->q_ll_2_body = s_data->q_body_2_ll.conjugate(); 
	
	// Linear Accelerations - Body to LL
	acc_ll = s_data->q_body_2_ll.rotate(acc_body); 
	s_data->x_ddot_ll = acc_ll.x; // X
	s_data->y_ddot_ll = acc_ll.y; // Y
	s_data->z_ddot_ll = acc_ll.z; // Z
	
	// Linear Velocities - Body to LL
	vel_ll = s_data->q_body_2_ll.rotate(vel_body); 
	s_data->x_dot_ll = vel_ll.x; // X
	s_data->y_dot_ll = vel_ll.y; // Y
	s_data->z_dot_ll = vel_ll.z; // Z
	
	// Linear Position - Body to LL
	pos_ll = s_data->q_body_2_ll.rotate(pos_body); 
	s_data->x_ll = pos_ll.x; // X
	s_data->y_ll = pos_ll.y; // Y
	s_data->z_ll = acc_ll.z; // Z
	
	// Extract Euler Angles 
	eulers = s_data->q_body_2_ll.extract_eulers(); 
	s_data->roll_rad = eulers.x; 
	s_data->pitch_rad = eulers.y; 
	s_data->yaw_rad = eulers.z;
	s_data->roll_deg = eulers.x * (180/M_PI); 
	s_data->pitch_deg = eulers.y * (180/M_PI);  
	s_data->yaw_deg = eulers.z * (180/M_PI);
	
}
