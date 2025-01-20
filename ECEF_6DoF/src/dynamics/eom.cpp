// This Script Contains the 1 Cycle Calculation of a 6DoF EOM in the Body Frame

// THESE MUST BE INTEGRATED AFTER OUTPUT FROM THIS ROUTINE - Simulaneous Diff Eq. 

// Inputs are in BODY FRAME 

#include <cmath> // includes a lot of the standard libraries 
#include "./eom.h"

void eqmot(const Vector& forces, const Vector& moments, const Vector& gravity_ned, const Vector& vel_body, const Vector& rad_earth,  
const Vector& rot_speed_earth_inertial, const Vector& rots_body, const Matrix3x3& i_inv, const Matrix3x3& i, 
const Vector& rot_speed_earth_body, double mass, double output[6])
{ 

// Equations of motion with Consideration for Rotational Speed of Earth and Everything 

// TRANSLATIONAL EQUATIONS //

//     v_dot    =    ( f * 1/m + g ) - (w_body cross vel_body) 
Vector v_dot = (forces.scalar_mult(1/mass).add_vector(gravity_ned)).subtract_vector(rots_body.cross( vel_body ))
			// - w_earth_body cross vel_body
			.subtract_vector(rot_speed_earth_body.cross(vel_body))
			// - w_earth_body cross w_earth_inertial cross earth_radius
			.subtract_vector(rot_speed_earth_body.cross(rot_speed_earth_inertial.cross(rad_earth))); 

// ROTATIONAL EQUATIONS //

//     w_dot = I^-1 * (moments - w_body cross I * w_body)
Vector w_dot = i_inv.MatrixMult(moments.subtract_vector(rots_body.cross(i.MatrixMult(rots_body)))); 

output[1] = v_dot.x;
output[2] = v_dot.y; 
output[3] = v_dot.z; 
output[4] = w_dot.x; 
output[5] = w_dot.y;
output[6] = w_dot.z; 

}
