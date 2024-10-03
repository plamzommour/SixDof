/* Quaternion Main Operational Script

This one uses quaternion.h to create lots of cool math for use later in another program

Available operations - Creation, Normalization, Multiplication and Rotation 
*/ 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./quaternion.h" 

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
        w * q.z + x * q.y - y * q.x - z * q.w
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
	 // 0.5  * (-rot_rates_body[0]*quat_in[1] - rot_rates_body[1]*quat_in[2] - rot_rates_body[2]*quat_in[3])
	 0.5  * (-v.x*x - v.y*y - v.z*z),
	 // 0.5  * (rot_rates_body[0]*quat_in[0] + rot_rates_body[2]*quat_in[2] - rot_rates_body[1]*quat_in[3]);
	 0.5  * (v.x*w + v.z*y - v.y*z), 
	 // 0.5  * (rot_rates_body[1]*quat_in[0] - rot_rates_body[2]*quat_in[1] + rot_rates_body[0]*quat_in[3])
	 0.5  * (v.y*w - v.z*x + v.x*z), 
	 // 0.5  * (rot_rates_body[2]*quat_in[0] + rot_rates_body[1]*quat_in[1] - rot_rates_body[0]*quat_in[2])
	 0.5  * (v.z*w + v.y*x - v.x*y)
	 );
	 
}
