/* Quaternion Main Operational Script

This one uses quaternion.h to create lots of cool math for use later in another program

Available operations - Creation, Normalization, Multiplication and Rotation 
*/ 

#include <cmath> // includes a lot of the standard libraries 
#include "./quaternion.h" 
#include "../dynamics/eom.h"

// Normalization 
void Quaternion::normalize()
{ 
	double n = std::sqrt(w*w + x*x + y*y + z*z); 
	w /= n; 
	x /= n; 
	y /= n; 
	z /= n; 
	
}

// Need a conjugation function 
Quaternion Quaternion::conjugate() const
{ 
	return Quaternion(w, -x, -y, -z); 
	
}  
	
// Multiplication
Quaternion Quaternion::quat_mult(const Quaternion& q) const
{ 
	return Quaternion
	( 
	w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w // w
        );
      
}
	
// Need an Operator for Rotation -- NORMALIZE FIRST
Vector Quaternion::rotate(const Vector& v) const
{ 
	Quaternion pureQ(0, v.x, v.y, v.z); // Pure Quaternion
	Quaternion rotVec = (*this).quat_mult(pureQ).quat_mult(this->conjugate()); // Rotation Formula
	return Vector(rotVec.x, rotVec.y, rotVec.z); // Return Rotated Vector
	
} 

// Quaternion Derivative - Active Rotation - Input is Angular Rates
Quaternion Quaternion::q_dot(const Vector& v) const
{ 

	return Quaternion
	( 
	 // 0.5  * (-rot_rates_body[0]*q.x - rot_rates_body[1]*q.y - rot_rates_body[2]*q.z)
	 0.5  * (-v.x*x - v.y*y - v.z*z),
	 // 0.5  * (rot_rates_body[0]*q.w + rot_rates_body[2]*q.y - rot_rates_body[1]*q.z);
	 0.5  * (v.x*w + v.z*y - v.y*z), 
	 // 0.5  * (rot_rates_body[1]*q.w - rot_rates_body[2]*q.x + rot_rates_body[0]*q.z)
	 0.5  * (v.y*w - v.z*x + v.x*z), 
	 // 0.5  * (rot_rates_body[2]*q.w + rot_rates_body[1]*q.x - rot_rates_body[0]*q.y)
	 0.5  * (v.z*w + v.y*x - v.x*y)
	 );
	 
}

// Function to Extract Euler Angles from Body Frame to Local Level Quaternion 
Vector Quaternion::Extract_Eulers(const Quaternion& q) const
{ 

	double phi_atan2_term1; 
	double phi_atan2_term2; 
	double theta_asin_term; 
	double psi_atan2_term1; 
	double psi_atan2_term2; 

	// Calculate Components of Formulas for Shorthand 
	phi_atan2_term1 = 2 * (q.w*q.x + q.y*q.z); 
	phi_atan2_term2 = (q.w*q.w + q.z*q.z - q.x*q.x - q.y*q.y); 
	theta_asin_term = 2 * (q.w*q.y - q.x*q.z);
	psi_atan2_term1 = 2 * (q.w*q.z + q.x*q.y); 
	psi_atan2_term2 = (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z); 	 
	
	// TODO - Add ArcSine Protection to Pitch Angle 
	
	return Vector (atan2(phi_atan2_term1, phi_atan2_term2),  
		       asin(theta_asin_term), 
	               atan2(psi_atan2_term1, psi_atan2_term2)); 

} 

