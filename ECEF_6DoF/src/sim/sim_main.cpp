// Sim Main Function

#include <cmath>
#include "../common/variables.h"
#include "./sim_init.h"
#include "../dynamics/dynamics_driver.h"
#include "../kinematics/initialize_quaternions.h"
#include "../kinematics/kinematics_driver.h"

int main ()
{ 

	const double t_end = 100; 
	const int int_rate = 1000; 
	
	// Define Data Structure and Initialize it 
	comvar sim_data; 
	// Set it to a pointer to use from here on out
	comvar *s_data = &sim_data; 
	
	// Initialize Data to Zeros 
	initialize_data(s_data);  

	// Find Initial Conditions from Input Files 
	// TO-DO - Input Deck Extraction 

	// Initialize Quaternions
	quaternion_init(s_data); 

	// Put Simulation Integration Rate Here 
	s_data->dt = 1; 
	s_data->dt = s_data->dt / int_rate; 
	
	// Enter Runtime Loop 
	while (s_data->time < t_end)
	{ 
	
	// Calculate Aero and GNC for One Cycle 
	

	// Calculate EoM Diff EQ for One Cycle 
	// This also runs the body-frame integration for rate and position. 
	// s_data will now be filled with current body-frame states 
	dynamics(s_data); 

	// Run Kinematics Driver.  Kinematics Driver Figures Out All Orientations, 
	// Performs the Quaternion Derivative, Calculates the LLA Position, Finds the New 
	// LL2ECEF Quaternion and Extracts the Euler Angles
	kinematics(s_data); 

	// Set Timer 
	s_data->time = s_data->time + s_data->dt; 

	// Stop if you hit the ground, or stop after landing 
	
	// Add an Eval Stop Function
	
	} 
	
 	return 0; 
 	
}; 
