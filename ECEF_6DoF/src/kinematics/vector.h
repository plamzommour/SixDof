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
	
	/* 
	This is a constructor for a class named Vector. It initializes a Vector object with three double-precision floating-point 
	values: _x, _y, and _z. If no values are provided, they default to 0.0.
	*/
	
	Vector(double _x = 0.0, double _y = 0.0, double _z = 0.0)
		: x(_x), y(_y), z(_z) {}
		
	// Vector to Vector Addition 
	/* const Vector& v (Parameter)

    	const → The parameter v is read-only (cannot be modified inside the function).
   	 Vector& → It is passed by reference to avoid unnecessary copying.

	const (After the Parameter List)

   	 Declares that add_vector does not modify the current object (this pointer is treated as const). */
   	 
	Vector add_vector(const Vector& v) const; 
	
	// Vector to Vector Subtraction
	Vector subtract_vector(const Vector& v) const; 
	
	// Vector to Scalar Multiplication 
	Vector scalar_mult(double scalar) const; 
	
	// Vector to Vector Cross Product 
	Vector cross(const Vector& v) const; 
		
}; 

#endif
