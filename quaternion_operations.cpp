// This is the script that actually does the heavy lifting for the quaternion rotation

// The goal of this is to rotate vect_in between inertial frame and body frame

// Please see the reference : en.wikipedia.org/wiki/Conversion_between_quaternions_euler_angles

/* 

Define quaternion, then turn into DCM.

TO-DO, see if q*v*q_INV can work too. 

Vector Math Functions Needed: 

vector_math.cross_product
vector_math.vector_add
vector_math.vector_subtract
vector_math.matrix_scalar_multiply

Note : Euler angles must be in RADIANS

Another Note: To reverse quaternion rotation, invert the complex part

*/ 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "matrix_operations.h"
#include "quaternion_operations.h"
using namespace N; 
using namespace Q; 
#define n 3

// Initialize Quaternion from Euler Angles
void quat::init_quaternion(double euler_angles[], double quat_rot[])
{

	double c_2_psi, s_2_psi, c_2_theta, s_2_theta, c_2_phi, s_2_phi; 
	
	// Take the half angles of the euler angle inputs 
	
	c_2_psi = cos(euler_angles[2]/2); 
	s_2_psi = sin(euler_angles[2]/2); 
	c_2_theta = cos(euler_angles[1]/2); 
	s_2_theta = sin(euler_angles[1]/2); 
	c_2_phi = cos(euler_angles[0]/2); 
	s_2_phi = sin(euler_angles[0]/2); 
	
	// Use those in the formula for the rotation unit quaternion - Init 3-2-1 to BODY
	
	quat_rot[0] = c_2_phi*c_2_theta*c_2_psi + s_2_phi*s_2_theta*s_2_psi; // q[w] - Scalar 
	quat_rot[1] = s_2_phi*c_2_theta*c_2_psi - c_2_phi*s_2_theta*s_2_psi; // q[0]
	quat_rot[2] = c_2_phi*s_2_theta*c_2_psi + s_2_phi*c_2_theta*s_2_psi; // q[1]
	quat_rot[3] = c_2_phi*c_2_theta*s_2_psi - s_2_phi*s_2_theta*c_2_psi; // q[2]
	
}

// Runtime Quaternion Rotation - Inertial to Body Frame 
void quat::quaternion_form_dcm(double quat_in[], double dcm[3][3])
{

	double e0_2, ex_2, ey_2, ez_2; 
	double e0, ex, ey, ez;   
	
	// First you gotta assign the values
	e0_2 = quat_in[0]*quat_in[0]; 
	ex_2 = quat_in[1]*quat_in[1]; 
	ey_2 = quat_in[2]*quat_in[2]; 
	ez_2 = quat_in[3]*quat_in[3]; 
	
	e0 = quat_in[0]; 
	ex = quat_in[1]; 
	ey = quat_in[2]; 
	ez = quat_in[3];
	
	// Then you gotta build the DCM
	dcm[0][0] = e0_2 + ex_2 - ey_2 - ez_2; 
	dcm[0][1] = 2*(ex*ey-ez*e0); 
	dcm[0][2] = 2*(e0*ey+ex*ez); 
	dcm[1][0] = 2*(ex*ey+ez*e0);
	dcm[1][1] = e0_2 - ex_2 + ey_2 - ez_2; 
	dcm[1][2] = 2*(ey*ez-ex*e0);
	dcm[2][0] = 2*(ex*ez-ey*e0);
	dcm[2][1] = 2*(e0*ex+ey*ez);
	dcm[2][2] = e0_2 - ex_2 - ey_2 + ez_2; 
	
	// This DCM will rotate a vector in BODY FRAME to the INERTIAL FRAME IN NED
	
}

// Rotate from Body Frame to Inertial Frame
void quat::rotate_B2I(double vector[], double dcm[3][3], double vector_out[])
{
	
	// v_1 = dcm*v; 
	vector_math::matrix_2_vect(dcm, vector, vector_out);

}

// Rotate from Inertial Frame to Body Frame 
void quat::rotate_I2B(double vector[], double dcm[3][3], double vector_out[])
{

	double t_dcm[3][3];
	
	// Transpose DCM 
	t_dcm[0][0] = dcm[0][0]; 
	t_dcm[0][1] = dcm[1][0];
	t_dcm[0][2] = dcm[2][0]; 
	t_dcm[1][0] = dcm[0][1];
	t_dcm[1][1] = dcm[1][1]; 
	t_dcm[1][2] = dcm[2][1]; 
	t_dcm[2][0] = dcm[0][2]; 
	t_dcm[2][1] = dcm[1][2];
	t_dcm[2][2] = dcm[2][2]; 
	
	// v_1 =  t_dcm*v
	vector_math::matrix_2_vect(t_dcm, vector, vector_out); 

}

// Propogate Quaternion to Quat_Dot -- May need feedback on this if it keeps blowing up
void quat::propogate_quaternion(double quat_in[], double rot_rates_body[], double quat_dot_out[])
{

	quat_dot_out[0] = 0.5  * (-rot_rates_body[0]*quat_in[1] - rot_rates_body[1]*quat_in[2] - rot_rates_body[2]*quat_in[3]); 
	quat_dot_out[1] = 0.5  * (rot_rates_body[0]*quat_in[0] + rot_rates_body[2]*quat_in[2] - rot_rates_body[1]*quat_in[3]); 
	quat_dot_out[2] = 0.5  * (rot_rates_body[1]*quat_in[0] - rot_rates_body[2]*quat_in[1] + rot_rates_body[0]*quat_in[3]); 
	quat_dot_out[3] = 0.5  * (rot_rates_body[2]*quat_in[0] + rot_rates_body[1]*quat_in[1] - rot_rates_body[0]*quat_in[2]); 

}

// Find Euler Angles from Quaternion - Can go Either Way (I think) 
void quat::find_eulers(double quat_in[], double eulers_out[]) 
{
	
	double phi_atan2_term1; 
	double phi_atan2_term2; 
	double theta_asin_term; 
	double psi_atan2_term1; 
	double psi_atan2_term2; 
	
	// Set Up Pieces
	phi_atan2_term1 = 2 * (quat_in[0]*quat_in[1] + quat_in[2]*quat_in[3]); 
	phi_atan2_term2 = (quat_in[0]*quat_in[0] + quat_in[3]*quat_in[3] - quat_in[1]*quat_in[1] - quat_in[2]*quat_in[2]); 
	theta_asin_term = 2 * (quat_in[0]*quat_in[2] - quat_in[1]*quat_in[3]);
	psi_atan2_term1 = 2 * (quat_in[0]*quat_in[3] + quat_in[1]*quat_in[2]); 
	psi_atan2_term2 = (quat_in[0]*quat_in[0] + quat_in[1]*quat_in[1] - quat_in[2]*quat_in[2] - quat_in[3]*quat_in[3]); 	 
	
	eulers_out[0] = atan2(phi_atan2_term1, phi_atan2_term2); 
	eulers_out[1] = asin(theta_asin_term); 
	eulers_out[2] = atan2(psi_atan2_term1, psi_atan2_term2); 

}

// Rotate Angular Velocity to NED from Body Frame using Euler Convention. Not a Quaternion Operation but Close Enough
void quat::omega_2_euler_rates(double omega_in[], double euler_in[], double euler_rates[])
{

	double matrix[3][3];
	
	// Angular Velocity Body to Eulers Formula
	matrix[0][0] = 1; 
	matrix[0][1] = sin(euler_in[0]) * tan(euler_in[1]); 
	matrix[0][2] = cos(euler_in[0]) * tan(euler_in[1]); 
	matrix[1][0] = 0; 
	matrix[1][1] = cos(euler_in[0]);
	matrix[1][2] = -sin(euler_in[0]); 
	matrix[2][0] = 0; 
	matrix[2][1] = sin(euler_in[0])/cos(euler_in[1]); 
	matrix[2][2] = cos(euler_in[0])/cos(euler_in[1]); 
	
	// Mutiply Omega and Matrix to get Euler Rates in NED
	vector_math::matrix_2_vect(matrix, omega_in, euler_rates);

}

// Quaternion Multplication - The HAMILTONIAN
void quat::quat_mult(double q_1[], double q_2[], double result_quat[])
{

	result_quat[0] = q_1[0]*q_2[0]-q_1[1]*q_2[1]-q_1[2]*q_2[2]-q_1[3]*q_2[3]; 
	result_quat[1] = q_1[0]*q_2[1]+q_1[1]*q_2[0]+q_1[2]*q_2[3]-q_1[3]*q_2[2];
	result_quat[2] = q_1[0]*q_2[2]-q_1[1]*q_2[3]+q_1[2]*q_2[0]+q_1[3]*q_2[1];
	result_quat[3] = q_1[0]*q_2[3]+q_1[1]*q_2[2]-q_1[2]*q_2[1]-q_1[3]*q_2[0];
     
}

// Quaternion Normalization - Only Performed ONCE Per Iteration Cycle
void quat::normalize_quaternion(double quat_in[]) 
{ 

	double norm_quat; 
	
	norm_quat = sqrt(quat_in[0]*quat_in[0]+quat_in[1]*quat_in[1]+quat_in[2]*quat_in[2]+quat_in[3]*quat_in[3]); 
	
	for (int i; i<4; i++)
	{ 
	
	quat_in[i] = quat_in[i] / norm_quat; 
	
	}

}

// Add Normalization Check Here. 
