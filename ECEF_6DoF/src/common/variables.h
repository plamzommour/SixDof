// This is the Common Variable Structure

// This contains all the pertinent common data that should be plotted

#ifndef VARIABLES_H
#define VARIABLES_H
#include "../kinematics/quaternion.h"

struct comvar
{ 

	// BODY FRAME //
	
	// Translational Body Accelerations - meters_sec_sec
	double u_dot_body; // X
	double v_dot_body; // Y
	double w_dot_body; // Z
	
	// Rotational Body Accelerations - rad_sec_sec
	double p_dot_body; // X
	double q_dot_body; // Y
	double r_dot_body; // Z	
	
	// Translational Body Rates - meters_sec
	double u_body; // X
	double v_body; // Y
	double w_body; // Z
	
	// Rotational Body Rates - rad_sec
	double p_body; // X
	double q_body; // Y
	double r_body; // Z	
	
	// Translation - Body - meters_sec
	double x_body; // X
	double y_body; // Y
	double z_body; // Z
	
	// Attitude - Body - rad
	double x_rot_body; // X
	double y_rot_body; // Y
	double z_rot_body; // Z	
	
	// Forces Acting On Body 
	double axial_force_body; // X
	double side_force_body;  // Y
	double norm_force_body;  // Z
	
	// Moments Acting On Body
	double x_mom_body; 
	double y_mom_body; 
	double z_mom_body; 
	
	// LOCAL LEVEL (L.L) FRAME //
	
	// Translational L.L. Accelerations - meters_sec_sec
	double x_ddot_ll; // X
	double y_ddot_ll; // Y
	double z_ddot_ll; // Z
	
	// Translational L.L. Rates - meters_sec
	double x_dot_ll; // X
	double y_dot_ll; // Y
	double z_dot_ll; // Z
	
	// Translation - L.L. - meters
	double x_ll; // X
	double y_ll; // Y
	double z_ll; // Z
	
	// Attitude - Euler Angles - rad
	double roll_rad; // X
	double pitch_rad; // Y
	double yaw_rad; // Z	
	
	// Attitude - Euler Angles - deg
	double roll_deg; // X
	double pitch_deg; // Y
	double yaw_deg; // Z	
	
	// Earth-Centered, Earth-Fixed Frame //
	
	// Translational ECEF Accelerations - meters_sec_sec
	double x_ddot_ecef; // X
	double y_ddot_ecef; // Y
	double z_ddot_ecef; // Z
	
	// Translational ECEF Rates - meters_sec
	double x_dot_ecef; // X
	double y_dot_ecef; // Y
	double z_dot_ecef; // Z
	
	// Translation - ECEF - meters
	double x_ecef; // X
	double y_ecef; // Y
	double z_ecef; // Z
	
	// Earth Rotational Rate
	double w_earth_ecef; 
	
	// Earth-Centered Inertial Frame //
	
	// Translational ECI Accelerations - meters_sec_sec
	double x_ddot_eci; // X
	double y_ddot_eci; // Y
	double z_ddot_eci; // Z
	
	// Translational ECEF Rates - meters_sec
	double x_dot_eci; // X
	double y_dot_eci; // Y
	double z_dot_eci; // Z
	
	// Translation - ECEF - meters
	double x_eci; // X
	double y_eci; // Y
	double z_eci; // Z
	
	// Earth Rotational Rate
	double w_earth_eci; 
	
	// Geodesic Coordinates //
	
	// Navigation Information 
	double latitude; // rad 
	double longitude; // rad 
	double altitude; // rad 
	
	// Mass Properties 
	double mass; 
	double inertia[3][3]; 
	
	// Simulation Timer
	double time; 
	double dt; 
	
	// Integration Step Counter 
	int step_count; 
	
	// Attitude Kinematics //
	
	// Quaternion - Body Frame to Local Level
	Quaternion q_body_2_ll;
	
	// Quaternion - Local Level to ECEF
	Quaternion q_ll_2_ecef;
	
	// Quaternion Derivative
	Quaternion q_dot; 
	
}; 

#endif 	

	
	
	
	
	
	
