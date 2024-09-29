// This Script Contains the 1 Cycle Calculation of a 6DoF EOM in the Body Frame

// THESE MUST BE INTEGRATED AFTER OUTPUT FROM THIS ROUTINE

/* We need the following inputs: 


Translational Velocities: Velocity of Body along X Axis, Velocity of Body along Y Axis, Velocity of Body along Z Axis 

	Input as "Velocity_Body[3]" 

Forces: Force acting on Body X Axis, Force Acting on Body Y Axis, Force Acting on Body Z Axis 

	Input as "Forces_Body[3]"

Rotations: P (rotation rate about X axis), Q (rotation rate about Y axis) , R (rotation rate about Z axis)

	Input as "Rotations_Body[3]" 

Moments: L (Moment about x axis - Roll), M (Moment about y axis - Pitch), N (Moment about z axis - Yaw) 

	Input as "Moments_Body[3]" 

Inertia Tensor:  | ixx iyz ixz 
                   iyz iyy iyz
                   ixz iyz izz |
                   
	Input as "Inertia_Tensor[3][3]" 
	
Mass: Scalar Value of Mass 
	
	Input as "Current_Mass" 
	
Gravity Vector in Body Frame: 

	Input as Grav_in_Body[3]
	
Output is a 6 element matrix that contains the folllowing UNINTEGRATED Accelerations in BODY frame: 

	eom_out[0] = trans_x_body; U
	eom_out[1] = trans_y_body; V
	eom_out[2] = trans_z_body; W
	eom_out[3] = rot_x_body; P
	eom_out[4] = rot_y_body; Q
	eom_out[5] = rot_z_body; R
	
So, Lets begin this journey.  Assume all units are metric, all angles are in radians

*/

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "matrix_operations.h" // This includes all the linear algebra and vector math
#include "eom.h"
using namespace N; // Vector Math
using namespace E; // Equations of Motion
#define n 3
 
void eqmot::equations_of_motion(double velocity_body[], double forces_body[], double rotations_body[], double moments_body[], 
				double grav_in_body[], double current_mass, double inertia_tensor[3][3], double eom_out[]) 
{

	// Declare intermediate values below here: 
	
	double rots_cross_trans[n]; 		
	double forces_plus_gravity[n]; 
	double mass_inverse; 
	double forces_plus_gravity_times_inverse_mass[n]; 
	double trans_result[n]; 
	double inverse_inertia[n][n]; 
	double inertia_times_omega[n]; 
	double omega_cross_inertia_times_omega[n]; 
	double moments_minus_omega_cross_inertia_times_omega[n]; 
	double rots_result[n]; 
	
	// Calculate Translational Equations first
	
	// v_dot = 1/mass*(forces_body+gravity_body)-rotations_body [cross] velocity_body 
	
	// Calculate Translational Cross Product
	vector_math::cross_product(rotations_body, velocity_body, rots_cross_trans); 
	
	// Calculate Forces added to Gravity
	vector_math::vector_add(forces_body, grav_in_body, forces_plus_gravity); 
	
	// Multiply forces_plus gravity by scalar mass
	mass_inverse = 1/current_mass; 
	vector_math::matrix_scalar_multiply(mass_inverse, forces_plus_gravity, forces_plus_gravity_times_inverse_mass); 
	
	// Subtract rots_cross_trans from forces_plus_gravity_times_inverse_mass - This is the final result for translation. 
	vector_math::vector_subtract(forces_plus_gravity_times_inverse_mass, rots_cross_trans, trans_result); 
	
	// Then, Calculate Rotational Equations
	
	// omega_dot = inv_i * (moments - omega [cross] (i * omega)
	
	// Calculate inverse of inertia tensor
	vector_math::invert_matrix(inertia_tensor, inverse_inertia);
	
	// Calculate Inertia times Omega
	vector_math::matrix_2_vect(inertia_tensor, rotations_body, inertia_times_omega); 
	
	// Calculate Omega Cross Inertia_Times_Omega 
	vector_math::cross_product(rotations_body, inertia_times_omega, omega_cross_inertia_times_omega); 
	
	// Calculate Moments minus omega_cross_inertia_times_omega
	vector_math::vector_subtract(moments_body, omega_cross_inertia_times_omega, moments_minus_omega_cross_inertia_times_omega); 
	
	// Calculate inverted inertia tensor times moments_minus_omega_cross_inertia_times_omega
	vector_math::matrix_2_vect(inverse_inertia, moments_minus_omega_cross_inertia_times_omega, rots_result); 
	
	// Put it all in a 6 Number State Vector 
	eom_out[0] = trans_result[0]; // x axis translation - body 
	eom_out[1] = trans_result[1]; // y axis translation - body
	eom_out[2] = trans_result[2]; // z axis translation - body
	eom_out[3] = rots_result[0]; // x axis rotation - body 
	eom_out[4] = rots_result[1]; // y axis rotation - body 
	eom_out[5] = rots_result[2]; // z axis rotation - body
	
	// REMEMBER, these are in the BODY frame, and must be INTEGRATED
} 


