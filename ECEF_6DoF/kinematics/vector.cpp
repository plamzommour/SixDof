/* This is the Implementation script for a 3D vector and associated operations: 

	Vector Creation 
	Vector Addition 
	Vector Subtraction 
	Scalar Multiplication 
	Cross Product 
	Matrix to Vector Multiplication
	
	To use these - create a vector, then call: 
	Vector <variable> = v1.add_vector(v2); 
	
	These operations are called by the equations of motion
	
*/ 
#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./vector.h"


// Vector to Vector Addition 
Vector Vector::add_vector(const Vector& v) const
{ 
	return Vector
	( 
		x+v.x, 
		y+v.y, 
		z+v.z
	); 
	
}
	
// Vector to Vector Subtraction
Vector Vector::subtract_vector(const Vector& v) const
{ 
	return Vector
	( 
		x-v.x, 
		y-v.y, 
		z-v.z
	); 
	
}
	
// Vector to Scalar Multiplication 
Vector Vector::scalar_mult(double scalar) const
{ 
	return Vector
	( 
		x*scalar, 
		y*scalar, 
		z*scalar
	); 
	
}
	
// Vector to Vector Cross Product 
Vector Vector::cross(const Vector& v) const
{ 
	return Vector
	( 
		y*v.z - z*v.y, 
		z*v.x - x*v.z, 
		x*v.y - y*v.x
	); 
	
}



