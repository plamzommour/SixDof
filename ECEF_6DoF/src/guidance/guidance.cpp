// This Script Contains Various Guidance Algorithms for the GNC Portion of the Sim

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./guidance.h"
#include "./matrix_operations.h"

using namespace N; // Matrix Operations
using namespace GD; // Guidance 

/* 
Waypoint guidance: This algorithm takes in the position, euler angles and airspeed of the vehicle and outputs a bank angle. 
Algorithm calculates the bearing between two waypoints in NED space formulated in [downrange, crossrange] form. 
Algorithm calculates the bank angle needed to get the vehicle to that bearing via lateral acceleration calculated from a heading error
and then steers out any angular error to the intended bearing. 

Crosstrack distance is also considered. 
Crosstrack error is calculated by rotating the current position and waypoint to be relative to vertical and using the crosstrack component to calculate the error. 

Both the bearing-based bank command and crosstrack-based bank command are added to one another.  
Assumption is that if you are on the intended bearing and have no crosstrack error, you are on the exact path you need to be and errors will be ~0. 
*/

void guidance::waypoint_guidance(double position[], double eulers[], double alpha_beta_airspeed[], double &bank_required, double& heading_err) 
{ 
	
	double guidepoints[8][2] = {{0,0}, {7000, -10000}, {15000, -10000}, {22000, 0}, {15000, 10000}, {7000, 10000}, {0,0}, {-10000,0}}; 
	double x_err, y_err, x_pos_err, y_pos_err, turn_radius, ay_req, dist_2_waypoint, x_track_err; 
	double rot_mat_z[3][3]; 
	double guide_points1[3]; 
	double pos_rot[3]; 
	double pos[3]; 
	double guide_points1_rot[3]; 
	static double heading_2_gp; 
	static int waypoint_number = 1; 
	static int calc_heading = 1; 
	static int waypoint_has_been_cycled = 0; 
	int count; 
	
	// First Step - Establish Positional Error WRT to guidepoint -- We can get away with not using fancy formulas cause we are in flat-earth coordinates
	x_err = guidepoints[waypoint_number][0] - guidepoints[waypoint_number-1][0]; // downtrack error 
	y_err = guidepoints[waypoint_number][1] - guidepoints[waypoint_number-1][1]; // crosstrack error
	guidance::counter(count); 
	
	x_pos_err = guidepoints[waypoint_number][0] - position[0]; 
	y_pos_err = guidepoints[waypoint_number][1] - position[1];
	
	// Calculate Distance to Waypoint
	dist_2_waypoint = sqrt(x_pos_err*x_pos_err + y_pos_err*y_pos_err); 
	
	// Cycle Waypoints 
	// Cycle Waypoint if threshold is met
	if (dist_2_waypoint < 2000 && waypoint_has_been_cycled == 0)
	{ 
	waypoint_number += 1; // Cycle Waypoint
	waypoint_has_been_cycled = 1; 
	}
	if (dist_2_waypoint > 2000 && waypoint_has_been_cycled == 1)
	{ 
	waypoint_has_been_cycled = 0;
	calc_heading = 1;  
	}
	
	// Calculate Bearing to new waypoint from previous waypoint
	if (calc_heading == 1)
	{
	heading_2_gp = atan2(y_err,x_err); // heading to guide point 
	calc_heading = 0; 
	} 
	
	// Calculate Heading Error
	heading_err = heading_2_gp - eulers[2]; // heading_error
	
	// Protection for Wrapping 
	if (heading_err > M_PI) 
	{ 
	heading_err -= 2*M_PI; 
	} 
	else if (heading_err < -M_PI) 
	{ 
	heading_err += 2*M_PI; 
	} 
	
	turn_radius = ( alpha_beta_airspeed[2] * 13) / heading_err; // Turn Radius --- 13 was added as a time-required piece. Can be tuned. 
	
	// Third Step - Compute Required Lateral Acceleration v^2/r
	ay_req = (alpha_beta_airspeed[2]*alpha_beta_airspeed[2]) / turn_radius; 
	
	// Fourth Step - Calculate Bank Angle Required -- Compare to Gravity 
	bank_required = atan2(ay_req,9.81); // Positive if Heading_Err is Positive - Right Turn 
	
	// Secondly, add Second Bank Condition for Crosstrack Error
	// Need Rotation Matrix to get things into a mathematically easy form - Vertical
	rot_mat_z[0][0] = cos(heading_2_gp); 
	rot_mat_z[0][1] = sin(heading_2_gp); 
	rot_mat_z[0][2] = 0; 
	rot_mat_z[1][0] = -sin(heading_2_gp); 
	rot_mat_z[1][1] = cos(heading_2_gp); 
	rot_mat_z[1][2] = 0; 
	rot_mat_z[2][0] = 0; 
	rot_mat_z[2][1] = 0; 
	rot_mat_z[2][2] = 1; 
	
	// Also Need to Get the Current Guidepoint Path in 3-D - Assume Z is Zero
	guide_points1[0] = guidepoints[waypoint_number][0]; // Further Waypoint X
	guide_points1[1] = guidepoints[waypoint_number][1]; // Further Waypoint Y 
	guide_points1[2] = 0; // No Z
	
	pos[0] = position[0]; // Current NED x position 
	pos[1] = position[1]; // Current NED y position 
	pos[2] = 0; // No Z
	
	// Use Matrix Multiply to Rotate Points 
	vector_math::matrix_2_vect(rot_mat_z, guide_points1, guide_points1_rot); // Rotate Guidepoint, only need crosstrack value 
	vector_math::matrix_2_vect(rot_mat_z, pos, pos_rot);
	
	// Formulate Crosstrack Error 
	x_track_err = pos_rot[1] - guide_points1_rot[1]; 
	
	// Add Crosstrack Error-Based Bank Command to Bearing-Based Bank Command 
	// Crosstrack Based Bank Command is generated using a simple proportional controller
	bank_required += (x_track_err * (-0.015 * M_PI/180)); 
	
	// Apply Limiting to Make it a Reasonable Bank
	if ( (std::abs(bank_required) > (35 * (M_PI/180) ) ) && (bank_required < 0) )
	{
	
		bank_required = -35 * (M_PI/180); // Negative Case 
	
	}
	if ( (std::abs(bank_required) > (35 * (M_PI/180) ) ) && (bank_required > 0) )
	{
	
		bank_required = 35 * (M_PI/180); // Positive Case 
	
	}
}

void guidance::counter(int &out)
{ 

	static int count = 0; 
	++count; 
	out = count; 
	return; 
	
}
