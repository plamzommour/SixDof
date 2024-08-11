// This Script Contains Various Guidance Algorithms for the GNC Portion of the Sim

// Waypoint Guidance - Takes in Position and a Waypoint and outputs a bank command to send to the autopilot 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./guidance.h"
using namespace GD; // Guidance 

void guidance::waypoint_guidance(double position[], double eulers[], double alpha_beta_airspeed[], double &bank_required, double& dist_2_waypoint) 
{ 
	
	double guidepoints[4][2] = {{10000, -20000}, {30000, -20000}, {30000, 20000}, {10000, 30000}}; // Downtrack, Crosstrack
	double x_err, y_err, turn_radius, ay_req, heading_err; 
	static double t_req, heading_2_gp, abs_heading_err; 
	static int correction_has_been_applied = 0; 
	static int waypoint_number = 0; 
	static int calc_heading = 0; 
	static int waypoint_has_been_cycled = 1; 
	const double turn_rate = 4 * (M_PI/180); // Standard 5 deg per second turn rate for small aircraft
	int count; 
	
	// Give Distance Buffer to Guidepoint - Future Work 

	// First Step - Establish Positional Error WRT to guidepoint -- We can get away with not using fancy formulas cause we are in flat-earth coordinates
	y_err = guidepoints[waypoint_number][0] - position[0]; // downtrack error 
	x_err = guidepoints[waypoint_number][1] - position[1]; // crosstrack error
	guidance::counter(count); 
	
	// Calculate Distance to Waypoint
	dist_2_waypoint = sqrt(x_err*x_err + y_err*y_err); 
	
	// Cycle Waypoint if threshold is met -- Move this logic to external function if able
	if (dist_2_waypoint < 1000 && waypoint_has_been_cycled == 0)
	{ 
	waypoint_number += 1; // Cycle Waypoint
	waypoint_has_been_cycled = 1; 
	}
	if (dist_2_waypoint > 1500 && waypoint_has_been_cycled == 1)
	{ 
	waypoint_has_been_cycled = 0;
	calc_heading = 1; 
	correction_has_been_applied = 0;  
	}
	
	// Calculate Bearing to guidepoint from current position
	if (calc_heading == 1)
	{
	heading_2_gp = atan2(x_err,y_err); // heading to guide point 
	calc_heading = 0; 
	}
	// Apply heading correction 15000 feet out from waypoint.  
	else if (dist_2_waypoint < 15000 && correction_has_been_applied == 0)
	{
	heading_2_gp = atan2(x_err,y_err); // heading to guide point 
	correction_has_been_applied = 1; 
	}
	
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
	
	// Apply Limiting Cause Command is going to +-90 deg 
	
	if ( (std::abs(bank_required) > (30 * (M_PI/180) ) ) && (bank_required < 0) )
	{
	
		bank_required = -30 * (M_PI/180); // Negative Case 
	
	}
	if ( (std::abs(bank_required) > (30 * (M_PI/180) ) ) && (bank_required > 0) )
	{
	
		bank_required = 30 * (M_PI/180); // Positive Case 
	
	}
}

void guidance::counter(int &out)
{ 

	static int count = 0; 
	++count; 
	out = count; 
	return; 
	
}
