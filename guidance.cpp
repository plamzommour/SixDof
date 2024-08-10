// This Script Contains Various Guidance Algorithms for the GNC Portion of the Sim

// Waypoint Guidance - Takes in Position and a Waypoint and outputs a bank command to send to the autopilot 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./guidance.h"
using namespace GD; // Guidance 

void guidance::waypoint_guidance(double position[], double eulers[], double alpha_beta_airspeed[], double &bank_required) 
{ 
	double guidepoints[2] = {10000, 10000}; 
	double x_err, y_err, heading_2_gp, heading_err, turn_radius, ay_req, abs_heading_err; 
	static double t_req; 
	const double turn_rate = 3 * (M_PI/180); // Standard 3 deg per second turn rate for small aircraft
	int count; 
	
	// Call guidepoint Sequencer - Establish Position Relative to Guidepoint. If you have passed, cycle to next waypoint - Future work 
	
	// Give Distance Buffer to Guidepoint - Future Work 

	// First Step - Establish Positional Error WRT to guidepoint -- We can get away with not using fancy formulas cause we are in flat-earth coordinates
	x_err = guidepoints[0] - position[0]; // downtrack error 
	y_err = guidepoints[1] - position[1]; // crosstrack error
	heading_2_gp = atan2(x_err,y_err); // heading to guide point 
	heading_err = heading_2_gp - eulers[2]; // heading_error 
	
	// Add protection for negative heading_error -- Bank Command is Re-Negated Later 
	abs_heading_err = std::abs(heading_err); 
	
	// Add protection for small angles of heading error 
	if (abs_heading_err < 0.00000000000000000000000000001)
	{
	
		abs_heading_err = 0.00000000000000000000000000001; 
		
	}
	
	// Second Step - Compute Turn Radius Needed 
	guidance::counter(count); 
	
	if (count == 1)
	{
		t_req = abs_heading_err/turn_rate; // ONLY DO THIS ONCE, or you will not get closure on bearing  
	}
	
	turn_radius = ( alpha_beta_airspeed[2] * t_req ) / abs_heading_err; // Turn Radius 
	
	// Third Step - Compute Required Lateral Acceleration
	ay_req = (alpha_beta_airspeed[2]*alpha_beta_airspeed[2]) / turn_radius; 
	
	// Fourth Step - Calculate Bank Angle Required -- Compare to Gravity 
	bank_required = atan2(9.81,ay_req); // Positive if Heading_Err is Positive - Right Turn 
	
	if (heading_err < 0)
	{ 
	
		bank_required = -bank_required; // Negative if Heading_Err is Negative - Left Turn 
		
	}
	
	// Apply Limiting Cause Command is going to +-90 deg 
	
	
}

void guidance::counter(int &out)
{ 

	static int count = 0; 
	++count; 
	out = count; 
	return; 
	
}
