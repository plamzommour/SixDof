/* This is a code that contains some simple autopilots for the lateral channel and vertical channel

Vertical channel controls the pitch by trying to maintain a certain vertical command, as given by the guidance. 

- Elevator Deflection is limited 
- Commanding action is performed on vertical acceleration and altitude 

// Feedback terms - Pitch rate, Theta, Vertical Acceleration (in NED) 
// Elevator command

Lateral channel controls the roll and yaw to either maintain a heading or to steer out a crosstrack error from guidance. 
Turns are coordinated by attempting to maintain zero sideslip. 

- Aileron deflection is limited 
- Rudder deflection is limited 

// Feedback Terms - Roll Rate, Yaw Rate, Phi, Psi, Beta
// Outputs - Aileron Command, Yaw Command 

*/ 
#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./autopilot.h"
using namespace AP; // Autopilot 

void autopilot::vertical_channel(double guidance_in[], double dynamics_in[], double int_path[], double dt, double &del_elevator)
{ 

	/* 
		autopilot diagram in general : 
						  \----Error * kP -------------|
	 	-------guidance cmd------+ --------				-------Deflection Command-------Dynamics---
	 			        - 	  |----Error += Error*dt*Ki----|				          |
	 			         \										  |
	 			         \								   		  |
	 			         \	                                                                          |
	 			         \----------------Feedback From 6DoF-----------------------------------------------
		
	*/
	
	double errors[3];  
	double prop_path[3]; 
	double pi_out[3]; 
	const double k_p = 20.0; 
	const double k_i = 1.0;
	const double el_scale = 0.5; 
	const double el_def_lim = 25*(M_PI/180); // 25 degree deflection limit 
	
	// Assume states coming in are theta, pitch rate and vertical acceleration, in the following way: 
	// [theta, q, nz] 
	
	// Formulate Error Signals 
	errors[0] = guidance_in[0] - dynamics_in[0]; // Theta ----- NEGATIVE Means that Elevator must go DOWN
	errors[1] = guidance_in[1] - dynamics_in[1]; // Pitch Rate 
	errors[2] = guidance_in[2] - dynamics_in[2]; // Altitude 
	
	// Formulate Proportional and Integral Paths and add them together 
	for (int i=0; i<3; i++)
	{
	int_path[i] += errors[i]*dt; 
	prop_path[i] = errors[i];
	pi_out[i] = prop_path[i]*k_p + int_path[i]*k_i;
	} 
	
	// Scale PI out for Elevator Deflection - Only controlling on theta feedback for now. 
	del_elevator = ( pi_out[0] ) * -el_scale; 
	
	// Apply Elevator Deflection Limits -- +- 25 deg
	if (del_elevator<-el_def_lim)
	{ 
	
		del_elevator = -el_def_lim; 
	
	}
	else if (del_elevator>el_def_lim)
	{
	
		del_elevator = el_def_lim; 
		
	} 

} 

void autopilot::lateral_channel(double guidance_in[], double dynamics_in[], double int_path[], double dt, double &del_rud, double &del_ail)
{ 

	/* 
		autopilot diagram in general : 
						  \----Error * kP -------------|
	 	-------guidance cmd------+ --------				-------Deflection Command-------Dynamics---
	 			        - 	  |----Error += Error*dt*Ki----|				          |
	 			         \										  |
	 			         \								   		  |
	 			         \	                                                                          |
	 			         \----------------Feedback From 6DoF-----------------------------------------------
		
	*/
	
	double errors[2]; 
	double prop_path[2]; 
	double pi_out[2]; 
	const double k_p = 0.5; 
	const double k_i = 1.0; 
	const double rud_scale = 0.00; // Rudder is off for now 
	const double rud_def_lim = 25*(M_PI/180); // 25 degree deflection limit 
	const double ail_scale = 0.40; 
	const double ail_def_lim = 25*(M_PI/180); // 25 degree deflection limit 
	
	// Assume states coming in are theta, pitch rate and vertical acceleration, in the following way: 
	// [theta, q, nz] 
	
	// Formulate Error Signals 
	errors[0] = guidance_in[0] - dynamics_in[0]; // Phi -- Positive means right bank 
	errors[1] = guidance_in[1] - dynamics_in[1]; // P about X - Body Roll Rate 
	
	// Formulate Proportional and Integral Paths and add them together 
	for (int i=0; i<2; i++)
	{
	int_path[i] += errors[i]*dt; 
	prop_path[i] = errors[i];
	pi_out[i] = prop_path[i]*k_p + int_path[i]*k_i;
	} 
	
	// Scale PI out for Rudder Deflection 
	del_rud = pi_out[0] * rud_scale; 
	
	// Apply Rudder Deflection Limits -- +- 25 deg
	if (del_rud<-rud_def_lim)
	{ 
	
		del_rud = -rud_def_lim; 
	
	}
	else if (del_rud>rud_def_lim)
	{
	
		del_rud = rud_def_lim; 
		
	} 
	
	// Scale PI out for aileron Deflection 
	del_ail = pi_out[0] * -ail_scale; 
	
	// Apply Rudder Deflection Limits -- +- 25 deg
	if (del_ail<-ail_def_lim)
	{ 
	
		del_ail = -ail_def_lim; 
	
	}
	else if (del_ail>ail_def_lim)
	{
	
		del_ail = ail_def_lim; 
		
	} 


} 

void autopilot::throttle_channel(double guidance_in[], double dynamics_in[], double int_path[], double dt, double& throttle)
{ 

	/* 
		autopilot diagram in general : 
						  \----Error * kP -------------|
	 	-------guidance cmd------+ --------				-------Deflection Command-------Dynamics---
	 			        - 	  |----Error += Error*dt*Ki----|				          |
	 			         \										  |
	 			         \								   		  |
	 			         \	                                                                          |
	 			         \----------------Feedback From 6DoF-----------------------------------------------
		
	*/
	
	double errors[3];  
	double prop_path[3]; 
	double pi_out[3]; 
	const double k_p = 15; 
	const double k_i = 0.1;
	const double throttle_scale = 0.01; 
	const double throttle_lower = 0.1; 
	const double throttle_upper = 1.0;
	
	// Assume states coming in are theta, pitch rate and vertical acceleration, in the following way: 
	// [theta, q, nz] 
	
	// Formulate Error Signals 
	errors[0] = guidance_in[0] - dynamics_in[0]; // Theta ----- NEGATIVE Means that Elevator must go DOWN
	errors[1] = guidance_in[1] - dynamics_in[1]; // Pitch Rate 
	errors[2] = guidance_in[2] - dynamics_in[2]; // Altitude 
	
	// Formulate Proportional and Integral Paths and add them together 
	for (int i=0; i<3; i++)
	{
	int_path[i] += errors[i]*dt; 
	prop_path[i] = errors[i];
	pi_out[i] = prop_path[i]*k_p + int_path[i]*k_i;
	} 
	
	// Scale PI out for throttle - Need to Command decimal percentages
	throttle =  pi_out[2] * -throttle_scale; 
	
	// Apply limiting
	if (throttle<throttle_lower)
	{ 
	
		throttle = throttle_lower; 
	
	}
	else if (throttle>throttle_upper)
	{
	
		throttle = throttle_upper; 
		
	} 

	
} 
