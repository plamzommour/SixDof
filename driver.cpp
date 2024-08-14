// This Script Runs the Equations of Motion, Aerodynamics and GNC Algorithms for Simulating A Controlled Airplane.
/* 

All kinematics are defined by quaternions, propagated through time. 

Reference frames are NED as seen by the body, the wind and the inertial NED (observer) frame
 
All integrations are euler's method

Equations of motion, aero model and kinematics are executed 1x per cycle

Simulation stops if you hit the ground

Simulation runs in metric units

*/ 

#include <bits/stdc++.h> 
#include "quaternion_operations.h" 
#include "matrix_operations.h" 
#include "eom.h"
#include "integrate.h"
#include "aero.h"
#define n 3

using namespace N; // Matrix Operations 
using namespace Q; // Quaternion Operations
using namespace E; // Equations of Motion 
using namespace FE; // Numerical Integration 
using namespace ARO; // Aero

int main() 
{ 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  Step 1: Intialize All Your Values

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double vel_body_ms[n]; 
	double force_body_n[n]; 
	double rot_body_radsec[n]; 
	double eulers_rad[n]; 
	double momen_body_nm[n]; 
        double grav_NED_mss[n]; 
        double grav_bod_mss[n]; 
        double mass_kg; 
        // Brick Inertia
        //double inertia[n][n] = {{0.0025682175, 0.00, 0.00}, 
	//	    		{0.00, 0.0084210111, 0.00}, 
	//            		{0.00, 0.00, 0.009754656}};   
	// Sphere Inertia
	//double inertia[n][n] = {{4.8809, 0.00, 0.00}, 
	//	    		{0.00, 4.8809, 0.00}, 
	//            		{0.00, 0.00, 4.8809}};   
	
	// Cessna 172 Inertia 
	double inertia[n][n] = {{1285.315, 0.00, 0.00}, 
		    		  {0.00, 1824.93, 0.00}, 
	            		 {0.00, 0.00,  2666.893}};   
        
        double eom_output_unint[6]; 
        double eom_prev[6]; 
        double vel_NED_ms[n]; 
        double position_NED_m[n]; 
        double linear_accels_ms2_body[n]; 
        double linear_accels_ms2_NED[n]; 
	double quaternion[4]; 
	double quat_dot[4]; 
	double dcm_i2b[3][3];
        double dt; 
        double time; 
        double t_end;
        double nz_ned_g; 
        double alpha_beta_airspeed[n]; // Aero Params Vector
        double bank_required, heading_err;
	
	// Velocities - Cruise Speed of Cessna 172 ~ 110 knots
	vel_body_ms[0] = 56; // X Velocity in M/S
	vel_body_ms[1] = 0.0; // Y Velocity in M/S
	vel_body_ms[2] = 0.0; // Z Velocity in M/S
	
	// Forces - Zero for Now
	force_body_n[0] = 2000; 
	force_body_n[1] = 0.00000; 
	force_body_n[2] = 0.00000; 
	
	// Rotation - Zero for now 
	rot_body_radsec[0] = 0.0000; 
	rot_body_radsec[1] = 0.0000; 
	rot_body_radsec[2] = 0.0000; 
	
	// Euler Angles - Phi, Theta and Psi in NED - 3 - 2 - 1 Rotation Order
	eulers_rad[0] = 0 * (M_PI/180); // Good
	eulers_rad[1] = 0 * (M_PI/180); // Good
	eulers_rad[2] = 0 * (M_PI/180); //Good 
	
	// Initialize Quaternion 
	quat::init_quaternion(eulers_rad, quaternion);
	
	// Moments - Zero for Now 
	momen_body_nm[0] = 0.00000; 
	momen_body_nm[1] = 0.00000; 
	momen_body_nm[2] = 0.00000; 
	
	// Define Mass 
	//mass_kg = 2.26795; // Kg - Brick
	
	//mass_kg = 14.5939; // Kg - Sphere
	
	mass_kg = 771.107; // Kg - Cessna 172
	
	// Gravity - Constant Gravitational FORCE in NED Z Direction
	grav_NED_mss[0] = 0.000; 
	grav_NED_mss[1] = 0.000; 
	grav_NED_mss[2] = 9.81*mass_kg; // Constant 
	
	// Rotate Gravity to Body Frame
	quat::quaternion_form_dcm(quaternion, dcm_i2b); 
	quat::rotate_I2B(grav_NED_mss, dcm_i2b, grav_bod_mss); 
	
	// NOTE: dcm_i2b and quaternion are now initialized and does not need to be recalculated until after first cycle of EOM
	            
	// Define Initial Position in NED
	position_NED_m[0] = 0.0000; 
	position_NED_m[1] = 0.0000; 
	position_NED_m[2] = -2000.0000;  

	// Define Time Step Size, dt
	dt = 0.002; // 500 hz - To Minimize Error -- Need to Convert to Runge Kutta 4 Eventually
	
	// Set intial time to zero
	time = 0; 
	
	// Set end time
	t_end = 1300;  
	
	// Initialize Accelerations for Autopilot - 1 g
	nz_ned_g = 1; 
	
	// Set EOM_Prev to Zeros for initial values
	for(int i=0; i<6; i++)
	{
		eom_prev[i] = 0.0000; 
		
	}
	
	eom_prev[0] = 56; // X Body Velocity Initial
	eom_prev[2] = 0; 
	
	// Open file
	std::ofstream file;
	file.open ("MyData.txt"); 
	
	std::cout<< "Running Simulation! Paul's 6DoF! (^ - ^)  \n" << std::endl; 
	std::cout<< "Please See MyData.txt for Output \n" << std::endl; 
	
	// Print Data File header 
	file << "Time,Position X_NED,Position Y_NED,Position Z_NED,Phi,Theta,Psi,U_Body,V_Body,W_Body,Alpha,Beta,Airspeed,VelNEDX,VelNEDY,VelNEDZ,Drag,ZForce,Bank_Cmd,Heading_err" <<  std::endl; 
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Step 2: Enter the main loop for the EoM and Integration
	
	// THIS IS THE RUNTIME LOOP!
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	while (time < t_end)
	{ 
	
	// AERO FORCES AND MOMENTS
	aero::aero_driver(position_NED_m, eulers_rad, nz_ned_g, dt, time, vel_body_ms, rot_body_radsec, alpha_beta_airspeed, force_body_n, momen_body_nm, 
	bank_required, heading_err);
	
	// Call Equations of Motion for ONE CYCLE
	eqmot::equations_of_motion(vel_body_ms, force_body_n, rot_body_radsec, momen_body_nm, grav_bod_mss, mass_kg, inertia, eom_output_unint); 
			
	/* Resultant BODY FRAME EoM Output BEFORE integration is the following: 
	eom_output_unint[0] = x translation acceleration in Body Frame
	eom_output_unint[1] = y translation acceleration in Body Frame 
	eom_output_unint[2] = z translation acceleration in Body Frame 
	eom_output_unint[3] = rotational acceleration about x in body frame
	eom_output_unint[4] = rotational acceleration about y in body frame
	eom_output_unint[5] = rotational acceleration about z in body frame	
	*/
	
	// Integrate EoM_Out to get Velocities 	
	// Set up inputs for the integrator and run
	integrator::euler_integrate_6(eom_prev, eom_output_unint, dt); 
	
	/* Resultant BODY FRAME EoM Output AFTER integration is the following: 
	eom_output_integrated[0] = x translation velocity in Body Frame
	eom_output_integrated[1] = y translation velocity in Body Frame 
	eom_output_integrated[2] = z translation velocity in Body Frame 
	eom_output_integrated[3] = rotational velocity about x in body frame
	eom_output_integrated[4] = rotational velocity about y in body frame
	eom_output_integrated[5] = rotational velocity about z in body frame
	*/
	
	// Save Integrated EoM Values off for the Next Cycle 
	for(int i=0; i<3; i++) 
	{ 
		// Translational Velocity - Body 
		vel_body_ms[i] = eom_prev[i]; 
		
		// Rotational Velocity - Body
		rot_body_radsec[i] = eom_prev[i+3]; 
		
		// Use this Loop to Reassign Linear Accels Too 
		linear_accels_ms2_body[i] = eom_output_unint[i]; 
		
	}
	
	////// Rotate EoM output from Body Frame to Inertial Frame by using Quaternions
	
	// Rotate Body Accelerations to Inertial Accelerations in NED
	quat::rotate_B2I(linear_accels_ms2_body, dcm_i2b, linear_accels_ms2_NED); 
	
	// Reassign 3rd Value of Acceleration to Nz and convert to G; 
	nz_ned_g = linear_accels_ms2_NED[2] / 9.81; 
	
	// Use Quaternions for Translational Velocities - Now Vx Vy and Vz - Function takes in dcm_i2b and transposes it
	quat::rotate_B2I(vel_body_ms, dcm_i2b, vel_NED_ms);
	
	// Propogate quaternion using Body Angular Rates
        quat::propogate_quaternion(quaternion, rot_body_radsec, quat_dot); 
        
        // Integrate Quat_Dot_b2i to get new quaternion
        integrator::euler_integrate_4(quaternion, quat_dot, dt);
        
        // Normalize new quaternion
        quat::normalize_quaternion(quaternion);
	
	////// Run Integration to find Positions and Convert Quat_B2I to Euler Angles 
	
	// Integration of velocity in NED for Position 
	integrator::euler_integrate_3(position_NED_m, vel_NED_ms, dt);
	
	// Quaternion to Eulers
	quat::find_eulers(quaternion, eulers_rad); 
	
	// Form New DCM for the Next Round
	quat::quaternion_form_dcm(quaternion, dcm_i2b); 
	
	// Get Gravity in Body Frame for Next Round
	quat::rotate_I2B(grav_NED_mss, dcm_i2b, grav_bod_mss); 
	
	// Data File CSV Output - 1 Line = One Cycle
	file << time << ","<< position_NED_m[0] << ","<< position_NED_m[1] << ","<< position_NED_m[2]<< "," << eulers_rad[0] << ","<< eulers_rad[1] << ","<< eulers_rad[2]
		<< "," << vel_body_ms[0] << "," << vel_body_ms[1] << "," << vel_body_ms[2] << "," << alpha_beta_airspeed[0] << "," << alpha_beta_airspeed[1] << "," << 
		alpha_beta_airspeed[2] << "," << vel_NED_ms[0] << "," << vel_NED_ms[1] << "," << vel_NED_ms[2] << "," << force_body_n[0] << "," << 
		force_body_n[2] + grav_bod_mss[2] << "," << bank_required << "," << heading_err << std::endl; 
		
	////// Check to see if you hit the ground or not 
	
	if (position_NED_m[2] > -0.5)
	{ 
	
		std::cout<< "You have hit the ground!  " << std::endl; 
		exit(0); 
		file.close(); 
	
	}
	
	// Move timer forward 
	time = time+dt;	
	
	}
	
	file.close(); 
	return 0; 
} 


