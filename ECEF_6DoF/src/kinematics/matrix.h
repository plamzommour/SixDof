// This is the script that creates matrices to be used later on
// Also includes definition for matrix inversion

#include <cmath> // includes a lot of the standard libraries 
#include "./vector.h"

class Matrix3x3
{ 
	public: 
	
	// 3x3 Matrix
	double m[3][3]; 
	
	// Constructors for This
	Matrix3x3(); 
	Matrix3x3(double mat[3][3]); 
	
	// Matrix to Vector Multiply 
	Vector MatrixMult(const Vector& v) const; 
	
	// Matrix Inversion 
	static void invert_matrix(double mat_2_inv[3][3], double inv_mat_out[3][3])

		{ 

			double det; 
			double inv_det; 
	
			// Use that one FORTRAN Code as a Basis : fortranwiki.org/fortran/show/Matrix+inversion
	
			// Suppose the Following Matrix : 
	
			//   |  mat[0][0], mat[0][1], mat[0][2] |
			//   |  mat[1][0], mat[1][1], mat[1][2] |
			//   |  mat[2][0], mat[2][1], mat[2][2] |
	
			// Step 1: Find the Determinant of the Matrix 
	
			det = ( mat_2_inv[0][0]*(mat_2_inv[1][1]*mat_2_inv[2][2]-mat_2_inv[2][1]*mat_2_inv[1][2])) 
			     -( mat_2_inv[0][1]*(mat_2_inv[1][0]*mat_2_inv[2][2]-mat_2_inv[2][0]*mat_2_inv[1][2]))
			     +( mat_2_inv[0][2]*(mat_2_inv[1][0]*mat_2_inv[2][1]-mat_2_inv[2][0]*mat_2_inv[1][1])); 
	
			// Step 2: Inverse the Determinant 
	
			inv_det = 1/det; 
	
			// Step 3: Calculate the Inverse (Inverse Determininant * Matrix Minors (with cofactors))
	
			/* 
			    mat[0][0] --> det | mat[1][1], mat[1][2] |
			                      | mat[2][1], mat[2][2] |
	                      
			*/
		        inv_mat_out[0][0] = inv_det *( mat_2_inv[1][1]*mat_2_inv[2][2] - mat_2_inv[2][1]*mat_2_inv[1][2] );
        
		        /* 
			    mat[1][0] --> det | mat[0][1], mat[0][2] |
			                      | mat[2][1], mat[2][2] |
	                      
			*/
			inv_mat_out[1][0] = -inv_det *( mat_2_inv[0][1]*mat_2_inv[2][2] - mat_2_inv[2][1]*mat_2_inv[0][2] );
	
			/* 
			    mat[2[]0] --> det | mat[0][1], mat[0][2] |
			                     |  mat[1][1], mat[1][2] |
	                      
			*/
			inv_mat_out[2][0] = inv_det  *( mat_2_inv[0][1]*mat_2_inv[1][2] - mat_2_inv[1][1]*mat_2_inv[0][2] ); 
	
			/* 
			    mat[0][1] --> det | mat[1][0], mat[1][2] |
			                      | mat[2][0], mat[2][2] |
	                      
			*/
			inv_mat_out[0][1] = -inv_det *( mat_2_inv[1][0]*mat_2_inv[2][2] - mat_2_inv[2][0]*mat_2_inv[1][2] );
			/* 
			    mat[1][1] --> det | mat[0][0], mat[0][2] |
			                      | mat[2][0], mat[2][2] |
	                      
			*/
	
			inv_mat_out[1][1] = inv_det  *( mat_2_inv[0][0]*mat_2_inv[2][2] - mat_2_inv[2][0]*mat_2_inv[0][2] );
			/* 
			    mat[2][1] --> det | mat[0][0], mat[0][2] |
			                      | mat[1][0], mat[1][2] |
	                      
			*/
			inv_mat_out[2][1] = -inv_det *( mat_2_inv[0][0]*mat_2_inv[1][2] - mat_2_inv[1][0]*mat_2_inv[0][2] );
			/* 
			    mat[0][2] --> det | mat[1][0], mat[1][1] |
			                      | mat[2][0], mat[2][1] |
	                      
			*/	
			inv_mat_out[0][2] = inv_det  *( mat_2_inv[1][0]*mat_2_inv[2][1] - mat_2_inv[2][0]*mat_2_inv[1][1] );
			/* 
			    mat[1][2] --> det | mat[0][0], mat[0][1] |
			                      | mat[2][0], mat[2][1] |
	                      
			*/
			inv_mat_out[1][2] = -inv_det *( mat_2_inv[0][0]*mat_2_inv[2][1] - mat_2_inv[2][0]*mat_2_inv[0][1] );
			/* 
			    mat[2][2] --> det | mat[0][0], mat[0][1] |
			                      | mat[1][0], mat[1][1] |
	                      
			*/
			inv_mat_out[2][2] = inv_det  *( mat_2_inv[0][0]*mat_2_inv[1][1] - mat_2_inv[1][0]*mat_2_inv[0][1] );

		}

}; 
