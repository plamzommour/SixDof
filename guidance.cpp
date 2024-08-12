// This Script Contains Various Guidance Algorithms for the GNC Portion of the Sim

// Waypoint Guidance - Takes in Position and a Waypoint and outputs a bank command to send to the autopilot 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./guidance.h"
#include "./matrix_operations.h"

using namespace N; // Matrix Operations
using namespace GD; // Guidance 

void guidance::waypoint_guidance(double position[], double eulers[], double alpha_beta_airspeed[], double &bank_required, double& heading_err) 
{ 
	
	double guidepoints[9][2] ={{0,0}, {10000, -20000}, {30000, -20000}, {30000, 20000}, {60000, 20000}, {60000, -20000}, {50000, -30000}, {10000, -40000}, {0,0}}; // Downtrack, Crosstrack
	double x_err, y_err, x_pos_err, y_pos_err, turn_radius, ay_req, dist_2_waypoint; 
	double rot_mat_z[3][3]; 
	double guide_points1[3]; 
	double pos_rot[3]; 
	double pos[3]; 
	double guide_points1_rot[3]; 
	double x_track_err; 
	static double t_req, heading_2_gp, abs_heading_err, yaw_prev; 
	static int correction_has_been_applied = 0; 
	static int waypoint_number = 1; 
	static int calc_heading = 1; 
	static int waypoint_has_been_cycled = 0; 
	const double turn_rate = 4 * (M_PI/180); // Standard 5 deg per second turn rate for small aircraft
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
	// Cycle Waypoint if threshold is met -- Move this logic to external function if able
	if (dist_2_waypoint < 2000 && waypoint_has_been_cycled == 0)
	{ 
	waypoint_number += 1; // Cycle Waypoint
	waypoint_has_been_cycled = 1; 
	}
	if (dist_2_waypoint > 2000 && waypoint_has_been_cycled == 1)
	{ 
	waypoint_has_been_cycled = 0;
	calc_heading = 1; 
	correction_has_been_applied = 0;  
	}
	
	// Calculate Bearing to guidepoint from current position
	if (calc_heading == 1)
	{
	heading_2_gp = atan2(y_err,x_err); // heading to guide point 
	calc_heading = 0; 
	} 
	
	// To Do - Add Sign Change Protection to Yaw
	heading_err = heading_2_gp - eulers[2]; // heading_error 
	
	// Add protection for negative heading_error -- Bank Command is Re-Negated Later 
	abs_heading_err = std::abs(heading_err); 
	
	// Add protection for small angles of heading error 
	if (abs_heading_err < 0.00000000000000000000000000001)
	{
	
		abs_heading_err = 0.00000000000000000000000000001; 
		
	}
	
	// Second Step - Compute Turn Radius Needed 
	
	if (count == 1)
	{
		t_req = abs_heading_err/turn_rate; // ONLY DO THIS ONCE, or you will not get closure on bearing  
	}
	
	turn_radius = ( alpha_beta_airspeed[2] * t_req ) / heading_err; // Turn Radius 
	
	// Third Step - Compute Required Lateral Acceleration
	ay_req = (alpha_beta_airspeed[2]*alpha_beta_airspeed[2]) / turn_radius; 
	
	// Fourth Step - Calculate Bank Angle Required -- Compare to Gravity 
	bank_required = atan2(ay_req,9.81); // Positive if Heading_Err is Positive - Right Turn 
	
	// Secondly, add Second Condition for Crosstrack Error
	// Need Rotation Matrix to get things into a mathematically easy form 
	
	// rot_mat_z = [cosd(bearing),sind(bearing), 0;-sind(bearing),cosd(bearing), 0; 0, 0, 1]
	rot_mat_z[0][0] = cos(heading_2_gp); 
	rot_mat_z[0][1] = sin(heading_2_gp); 
	rot_mat_z[0][2] = 0; 
	rot_mat_z[1][0] = -sin(heading_2_gp); 
	rot_mat_z[1][1] = cos(heading_2_gp); 
	rot_mat_z[1][2] = 0; 
	rot_mat_z[2][0] = 0; 
	rot_mat_z[2][1] = 0; 
	rot_mat_z[2][2] = 1; 
	
	// Also Need to Get the Current Guidpoint Path in 3-D - Assume Z is Zero
	guide_points1[0] = guidepoints[waypoint_number][0]; // Further Waypoint X
	guide_points1[1] = guidepoints[waypoint_number][1]; // Further Waypoint Y 
	guide_points1[2] = 0; // No Z
	
	pos[0] = position[0]; // Current NED x position 
	pos[1] = position[1]; // Current NED y position 
	pos[2] = 0; // No Z
	
	// Use Matrix Multiply to Rotate Points 
	vector_math::matrix_2_vect(rot_mat_z, guide_points1, guide_points1_rot); // Rotate Guidepoint, only need crosstrack value 
	vector_math::matrix_2_vect(rot_mat_z, pos, pos_rot);
	
	// Forumulate Crosstrack Error 
	x_track_err = pos_rot[1] - guide_points1_rot[1];
	
	// Add Crosstrack Error to Bank Command with a gain after getting a little bit away
	bank_required += (x_track_err * (-0.015 * M_PI/180)); 
	
	// Apply Limiting Cause Command is going to +-90 deg 
	
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
