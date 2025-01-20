/* This is the header script for a 3D vector and associated operations: 
	Vector Creation 
	Vector Addition 
	Vector Subtraction 
	Scalar Multiplication 
	Cross Product 
	
*/ 

#include <cmath> // includes a lot of the standard libraries 
#ifndef VECTOR_H
#define VECTOR_H

class Vector
{ 
	public: 
	
	double x, y, z; 
	
	// Vector Intialization
	Vector(double _x = 0.0, double _y = 0.0, double _z = 0.0)
		: x(_x), y(_y), z(_z) {}
		
	// Vector to Vector Addition 
	Vector add_vector(const Vector& v) const; 
	
	// Vector to Vector Subtraction
	Vector subtract_vector(const Vector& v) const; 
	
	// Vector to Scalar Multiplication 
	Vector scalar_mult(double scalar) const; 
	
	// Vector to Vector Cross Product 
	Vector cross(const Vector& v) const; 
		
}; 

#endif
