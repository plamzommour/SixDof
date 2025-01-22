//Header file for quaternion operations

// This one is a little smarter than the last one 

// If we do this right, we should be able to use matlab-style syntax. 

// Chat GPT helped with this one 

#include <cmath> // includes a lot of the standard libraries 
#include "./vector.h"

class Quaternion 
{ 
	public: 
	double w, x, y, z; 
	
	// Need a constructor for our quaternion
	Quaternion(double _w = 1.0, double _x = 0.0, double _y = 0.0, double _z = 0.0)
		: w(_w), x(_x), y(_y), z(_z) {}
		
	// Need a void for Normalization 
	void normalize(); 
	
	// Define an Operator for Multiplication
	Quaternion quat_mult(const Quaternion& q) const; 
	
	// Need an Operator for Rotation 
	Vector rotate(const Vector& v) const; 
	
	// Need an Operator for Quaternion Derivative - Propogation
	Quaternion q_dot(const Vector& v) const; 
	
	// Need a conjugation function 
	Quaternion conjugate() const; 
	
	// Euler Angle Extraction
	Vector extract_eulers() const; 
	
}; 
