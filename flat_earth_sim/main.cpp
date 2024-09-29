// This is the Main Validation Script

// THIS DOES NOT RUN THE EOM 

#include "matrix_operations.h"
#include "quaternion_operations.h"
#include <bits/stdc++.h> 
#define n 3

// Validation Script 

using namespace N; 
using namespace Q; 

int main() 

{ 

double vector_1[] = {9.2, 3.2, 4.6}; 
double vector_2[] = {3.1, 3.2, 3.3}; 
double eulers_test[] = {0, (M_PI/8), (M_PI/7)}; //phi, theta, psi
double rot_vect[n]; 
double rot_vect_rev[n]; 
double cross_p[n]; 

double matrix_in[3][3]= { {116.00, 222.00, 3453.00}, 
			  {445.00, 555.00, 766.00}, 
			  {727.00, 8878.00, 996.00} };  
double matrix_inv[3][3]; 

double matrix_vect_mult[n]; 

double quat_out_b2i[4];
double quat_out_i2b[4];
double quat_check; 

std::cout<< "Cross Product: "<< std::endl; 

vector_math::cross_product(vector_1, vector_2, cross_p); 

for(int i=0; i<n; i++) 

	std::cout<< cross_p[i] << " " << std::endl; 

std::cout<< "Matrix Inversion: "<< std::endl; 

vector_math::invert_matrix(matrix_in, matrix_inv); 
	
for (int i=0; i<n; i++)
	for (int j=0; j<n; j++) 
	
	std::cout<< matrix_inv[i][j] << " " << std::endl; 
	
std::cout<< "Matrix Vector Product: "<< std::endl; 

vector_math::matrix_2_vect(matrix_in, vector_1, matrix_vect_mult); 

for(int i=0; i<n; i++) 

	std::cout<< matrix_vect_mult[i] << " " << std::endl; 
	
std::cout<< "Quaternion Rotation - Body to Inertial: "<< std::endl; 
	
quat::quaternion_rotate_B2I(vector_1, eulers_test, rot_vect, quat_out_b2i); 

for(int i=0; i<n; i++) 

	std::cout<< rot_vect[i] << " " << std::endl; 
	
std::cout<< "Quaternion Output - Body to Inertial: "<< std::endl; 

// check to see if the quaternion is normalized or close to it:

for(int i=0; i<4; i++) 

	std::cout<< quat_out_b2i[i] << " " << std::endl; 
	
quat_check = sqrt((quat_out_b2i[0]*quat_out_b2i[0])+(quat_out_b2i[1]*quat_out_b2i[1])+(quat_out_b2i[2]*quat_out_b2i[2])+(quat_out_b2i[3]*quat_out_b2i[3])); 

std::cout<< "Quaternion Output - Body to Inertial - Normalized Value = "<< quat_check << std::endl; 
	
std::cout<< "Quaternion Rotation - Inertial to Body: "<< std::endl; 
	
quat::quaternion_rotate_I2B(rot_vect, eulers_test, rot_vect_rev, quat_out_i2b); 

for(int i=0; i<n; i++) 

	std::cout<< rot_vect_rev[i] << " " << std::endl; 
	
return 0; 

}
