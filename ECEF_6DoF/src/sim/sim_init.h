// Sim Initialization - Initializes Common Var Init

#ifndef SIM_INIT_H
#define SIM_INIT_H

#include <cmath>
#include "../common/variables.h"

// Routine to Pre-Fill Everything with Zeros
void initialize_data(comvar* data)
{ 
	// BODY FRAME //
	
	// Translational Body Accelerations - meters_sec_sec
	data->u_dot_body = 0; // X
	data->v_dot_body = 0; // Y
	data->w_dot_body = 0; // Z
	
	// Rotational Body Accelerations - rad_sec_sec
	data->p_dot_body = 0; // X
	data->q_dot_body = 0; // Y
	data->r_dot_body = 0; // Z	
	
	// Translational Body Rates - meters_sec
	data->u_body = 0; // X
	data->v_body = 0; // Y
	data->w_body = 0; // Z
	
	// Rotational Body Rates - rad_sec
	data->p_body = 0; // X
	data->q_body = 0; // Y
	data->r_body = 0; // Z	
	
	// Translation - Body - meters_sec
	data->x_body = 0; // X
	data->y_body = 0; // Y
	data->z_body = 0; // Z
	
	// Attitude - Body - rad
	data->x_rot_body = 0; // X
	data->y_rot_body = 0; // Y
	data->z_rot_body = 0; // Z	
	
	// Forces Acting On Body 
	data->axial_force_body = 0; // X
	data->side_force_body = 0;  // Y
	data->norm_force_body = 0;  // Z
	
	// Moments Acting On Body
	data->x_mom_body = 0; 
	data->y_mom_body = 0; 
	data->z_mom_body = 0; 
	
	// LOCAL LEVEL (L.L) FRAME //
	
	// Translational L.L. Accelerations - meters_sec_sec
	data->x_ddot_ll = 0; // X
	data->y_ddot_ll = 0; // Y
	data->z_ddot_ll = 0; // Z
	
	// Translational L.L. Rates - meters_sec
	data->x_dot_ll = 0; // X
	data->y_dot_ll = 0; // Y
	data->z_dot_ll = 0; // Z
	
	// Translation - L.L. - meters
	data->x_ll = 0; // X
	data->y_ll = 0; // Y
	data->z_ll = 0; // Z
	
	// Attitude - Euler Angles - rad
	data->roll_rad = 0; // X
	data->pitch_rad = 0; // Y
	data->yaw_rad = 0; // Z	
	
	// Attitude - Euler Angles - deg
	data->roll_deg = 0; // X
	data->pitch_deg = 0; // Y
	data->yaw_deg = 0; // Z	
	
	// Earth-Centered, Earth-Fixed Frame //
	
	// Translational ECEF Accelerations - meters_sec_sec
	data->x_ddot_ecef = 0; // X
	data->y_ddot_ecef = 0; // Y
	data->z_ddot_ecef = 0; // Z
	
	// Translational ECEF Rates - meters_sec
	data->x_dot_ecef = 0; // X
	data->y_dot_ecef = 0; // Y
	data->z_dot_ecef = 0; // Z
	
	// Translation - ECEF - meters
	data->x_ecef = 0; // X
	data->y_ecef = 0; // Y
	data->z_ecef = 0; // Z
	
	// Earth Rotational Rate
	data->w_earth_ecef = 0; 
	
	// Earth-Centered Inertial Frame //
	
	// Translational ECI Accelerations - meters_sec_sec
	data->x_ddot_eci = 0; // X
	data->y_ddot_eci = 0; // Y
	data->z_ddot_eci = 0; // Z
	
	// Translational ECEF Rates - meters_sec
	data->x_dot_eci = 0; // X
	data->y_dot_eci = 0; // Y
	data->z_dot_eci = 0; // Z
	
	// Translation - ECEF - meters
	data->x_eci = 0; // X
	data->y_eci = 0; // Y
	data->z_eci = 0; // Z
	
	// Earth Rotational Rate
	data->w_earth_eci = 0; 
	
	// Geodesic Coordinates //
	
	// Navigation Information 
	data->latitude = 0; // rad 
	data->longitude = 0; // rad 
	data->altitude = 0; // rad 
	
	// Mass 
	data-> mass = 0; 
	data-> inertia[0][0] = 0;
	data-> inertia[0][1] = 0;
	data-> inertia[0][2] = 0;
	data-> inertia[1][0] = 0;
	data-> inertia[1][1] = 0;
	data-> inertia[1][2] = 0;
	data-> inertia[2][0] = 0;
	data-> inertia[2][1] = 0;
	data-> inertia[2][2] = 0;
	
	// Start timer at zero
	data->time = 0; 
	data->dt = 0; // Integration Rate; 
	
	// Start Integration Step Counter at Zero 
	data->step_count = 0; 
	
	// Attitude Kinematics //
	
	// Quaternion - Body Frame to Local Level
	Quaternion q_ll_2_body (0, 0, 0, 0);
	
	// Quaternion - Local Level to Body Frame
	Quaternion q_body_2_ll (0, 0, 0, 0);
	
	// Quaternion - Local Level to ECEF
	Quaternion q_ll_2_ecef (0, 0, 0, 0);
	
	// Quaternion Derivative
	Quaternion q_dot (0, 0, 0, 0); 
	
}

#endif
