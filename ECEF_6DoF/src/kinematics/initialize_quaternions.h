// This File is used to initialize the quaternions using euler angles and latitude and longitude. 

#ifndef INITIALIZE_QUATERNIONS_H
#define INITIALIZE_QUATERNIONS_H

#include <cmath>
#include "../common/variables.h"

void quaternion_init(comvar* s_data)
{ 
	
	// Local Body to Local Level Quaternions 
	Quaternion q_x_ll_2_body (cos(s_data->roll_rad/2), sin(s_data->roll_rad/2), 0, 0); 
	Quaternion q_y_ll_2_body (cos(s_data->pitch_rad/2), 0, sin(s_data->pitch_rad/2), 0); 
	Quaternion q_z_ll_2_body (cos(s_data->yaw_rad/2), 0, 0, sin(s_data->yaw_rad/2)); 
	Quaternion q_ll_2_body_result; 
	
	// Local Level to ECEF
//	Quaternion q_rot1_ecef_2_ll (cos(s_data->wp_longitude/2), 0, 0, sin(s_data->wp_longitude/2)); // Z Axis About Longitude 
//	Quaternion q_rot2_ecef_2_ll (cos(s_data->wp_latitude/2), 0, sin(s_data->wp_latitude/2), 0); // Y Axis About Latitude 
//	Quaternion q_ecef_2_ll_result; 
	
	// Construct Rotation Quaternion and Normalize it - Local Level to Body Frame
	// 3-2-1 Rotation Sequence 
	q_ll_2_body_result = q_z_ll_2_body.quat_mult(q_y_ll_2_body).quat_mult(q_x_ll_2_body); 
	q_ll_2_body_result.normalize();
	s_data->q_ll_2_body = q_ll_2_body_result; 
	s_data->q_body_2_ll = q_ll_2_body_result.conjugate(); 
	
	// Construct Rotation Quaternion and Normalize it - ECEF to Local Level
	// 3-2 Rotation Sequence 
//	q_ecef_2_ll_result = q_rot1_ecef_2_ll.quat_mult(q_rot2_ecef_2_ll); 
//	q_ecef_2_ll_result.normalize();
//	s_data->q_ecef_2_ll = q_ecef_2_ll_result; 
//	s_data->q_ll_2_ecef = q_ecef_2_ll_result.conjugate(); 
}
	
#endif 
