/* Implementation file for Matrix Operations

This script really only initializes the matrix and defines multiplication of a matrix and a vector

*/ 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "./matrix.h" 

Matrix3x3::Matrix3x3()
{ 
	for(int i=0; i<3; i++)
	{ 
		for (int j=0; j<3; j++)
		{ 
		m[i][j] = (i == j) ? 1.0000 : 0.0000; // Ternary operator - if i is Equal to j, use 1, else use 0
		}
	} 
	
} 

Matrix3x3::Matrix3x3(double mat[3][3])
{ 
	for(int i=0; i<3; i++)
	{ 
		for (int j=0; j<3; j++)
		{ 
		m[i][j] = mat[i][j]; 
		}
	} 
	
}

// Matrix Mutiply to Vector
Vector Matrix3x3::MatrixMult(const Vector& v) const
{ 
	return Vector
	( 
	m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z, 
	m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z, 
	m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
	); 
} 
	
	
