// Dynamics Driver 

// This Script is the Executive Script that Executes ONE Cycle of the ECEF Equations of Motion 

// This also calls the Integrator 

// Output is BODY FRAME Dynamics for ONE Cycle

#include <cmath>
#include "./eom.h" 
#include "./integrate.h"
#include "../common/variables.h"

void dynamics(comvar* s_data) 
{ 

	double inv_mat[3][3]; 
	double eom_out[6]; 
	double eom_prev[6]; 
	double eom_rates[6]; 
	double rates_prev[6]; 
	double eom_pos[6]; 
	
	// Bring in Common Variable Structure and Unpack the Dynamics
	
	// Pack Forces into BODY FRAME Vector Below Here 
	Vector forces; 
	forces.x = s_data->axial_force_body; 
	forces.y = s_data->side_force_body; 
	forces.z = s_data->norm_force_body; 
	
	// Pack Moments into BODY FRAME Vector Below Here
	Vector moments; 
	moments.x = s_data->x_mom_body; 
	moments.y = s_data->y_mom_body; 
	moments.z = s_data->z_mom_body; 
	
	// Pack Linear Velocities into BODY FRAME Vector Below Here 
	Vector vel_body; 
	vel_body.x = s_data->u_body; 
	vel_body.y = s_data->v_body; 
	vel_body.z = s_data->w_body; 
	
	// Pack Angular Velocities into BODY FRAME Vector Below Here
	Vector rots_body; 
	rots_body.x = s_data->p_body; 
	rots_body.y = s_data->q_body; 
	rots_body.z = s_data->r_body; 
	
	// Inertia Tensor from Mass Props 
	Matrix3x3 tensor(s_data->inertia); 
	
		// Unpack It, Invert it, Pack for EOM
		Matrix3x3::invert_matrix(s_data->inertia, inv_mat); 
		Matrix3x3 tensor_inv(inv_mat); 
	
	// Calculate Gravity in Body Frame -- TODO
	Vector gravity_ned; 
	gravity_ned.x = 0; 
	gravity_ned.y = 0; 
	gravity_ned.z = 0; 
	
	// Earth Calculations -- TODO 
	Vector rad_earth; 
	rad_earth.x = 0; 
	rad_earth.y = 0; 
	rad_earth.z = 0; 
	
	// Rotation Speed Earth - Inertial -- TODO
	Vector rot_speed_earth_inertial; 
	rot_speed_earth_inertial.x = 0; 
	rot_speed_earth_inertial.y = 0; 
	rot_speed_earth_inertial.z = 0; 
	
	// Rotation Speed Earth - Body -- TODO
	Vector rot_speed_earth_body; 
	rot_speed_earth_body.x = 0; 
	rot_speed_earth_body.y = 0; 
	rot_speed_earth_body.z = 0; 
	
	// Move Integration Step Counter 
	s_data->step_count = s_data->step_count + 1; 
	
	// Run one Cycle of the EoM Calculation 
	eqmot(forces, moments, gravity_ned, vel_body, rad_earth, rot_speed_earth_inertial, rots_body, tensor_inv, tensor, 
	rot_speed_earth_body, s_data->mass, eom_out);
	
	///// RATE INTEGRATION /////
	
	// Integrate First Step Using Euler's Method 
	// This sets up a "previous step" to be used in Adam/Bashforth method
	if (s_data->step_count == 1)
	{
	
		// Integrate Results for Body Frame Dynamics - Rates 
		for (int i=0; i<6; i++)
		{ 
			eom_rates[i] += eom_out[i] * s_data->dt; 
		
		}
	
	}	
	else 
	{
		
		// Unpack Previous Results 
		eom_prev[0] = s_data->u_dot_body;
		eom_prev[1] = s_data->v_dot_body;
		eom_prev[2] = s_data->w_dot_body;
		eom_prev[3] = s_data->p_dot_body;
		eom_prev[4] = s_data->q_dot_body;
		eom_prev[5] = s_data->r_dot_body;
		
		// Integrate Results for Body Frame Dynamics - Rates 
		for (int i=0; i<6; i++)
		{ 
			eom_rates[i] = AdamsBashforth(eom_out[i], eom_prev[i], s_data->dt); 
		
		}
	
	}
	
	
	///// POSITION INTEGRATION /////
	
	// Integrate First Step Using Euler's Method 
	if (s_data->step_count == 1)
	{
	
		// Integrate Results for Body Frame Dynamics - Rates 
		for (int i=0; i<6; i++)
		{ 
			eom_pos[i] += eom_rates[i] * s_data->dt; 
		
		}
		
	}	
	else 
	{
		
		// Unpack Previous Results 
		rates_prev[0] = s_data->u_body;
		rates_prev[1] = s_data->v_body;
		rates_prev[2] = s_data->w_body;
		rates_prev[3] = s_data->p_body;
		rates_prev[4] = s_data->q_body;
		rates_prev[5] = s_data->r_body;
		
		// Integrate Results for Body Frame Dynamics - Rates 
		for (int i=0; i<6; i++)
		{ 
			eom_rates[i] = AdamsBashforth(eom_rates[i], rates_prev[i], s_data->dt); 
		
		}
	
	}
	
	// Pack EoM Acceleration Outputs Into Structure - Is Now Previous Step Data
	
	s_data->u_dot_body = eom_out[0]; // X
	s_data->v_dot_body = eom_out[1]; // Y
	s_data->w_dot_body = eom_out[2]; // Z
	s_data->p_dot_body = eom_out[3]; // X
	s_data->q_dot_body = eom_out[4]; // Y
	s_data->r_dot_body = eom_out[5]; // Z	
	
	// Pack EoM Rates Outputs Into Structure - Is Now Previous Step Data
	
	s_data->u_body = eom_rates[0]; // X
	s_data->v_body = eom_rates[1]; // Y
	s_data->w_body = eom_rates[2]; // Z
	s_data->p_body = eom_rates[3]; // X
	s_data->q_body = eom_rates[4]; // Y
	s_data->r_body = eom_rates[5]; // Z	
	
	// Pack Positions Away for Next Cycle
	s_data->x_body = eom_pos[0]; 
	s_data->y_body = eom_pos[1]; 
	s_data->z_body = eom_pos[2]; 
	s_data->x_rot_body = eom_pos[3]; 
	s_data->y_rot_body = eom_pos[4]; 
	s_data->z_rot_body = eom_pos[5]; 
	
};
