// This file contains all the matrix and vector operations necessary for the equations of motion. 

// 1. Scalar Multiplication (Added)
// 2. Vector Multiplication 3x3 to 3x1 
// 3. Vector Subtraction (Added) 
// 4. Vector Addition (Added)
// 5. Cross Product (Added) 
// 6. Matrix Inversion (Added)
// 7. Matrix to a Matrix Multiplication -- Did not actually need due to usage of quaternions

// Initialize Script 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#define n 3

//Declare Namespace 
namespace N 
{ 
	class vector_math
	{
	
	public: 

		// 1. Scalar Multiplication 	
		static void matrix_scalar_multiply(double scalar, double mat_1_scalar[], double mat_result_scalar[])

		{ 
			for(int i=0; i<n; i++)
			{

				mat_result_scalar[i] = scalar*mat_1_scalar[i];  
	        
			}

		}

		// 2. Vector Multiplication to a 3x3 Matrix 
		static void matrix_2_vect(double mat_in_mult[3][3], double vect_in_mult[], double vect_out_mult[])
		{ 
			double vect_int[3][3]; 
	
			// Algorithm is: 
			/* 
			Vector = [ vect[0], vect[1], vect[2] ]
	
			Matrix =   |  mat[0][0], mat[0][1], mat[0][2] |
			//         |  mat[1][0], mat[1][1], mat[1][2] |
			//         |  mat[2][0], mat[2][1], mat[2][2] |
	
			Intermediate Matrix  =   |  vect[0]*mat[0][0], vect[1]*mat[0][1], vect[2]*mat[0][2] |
			//         		 |  vect[0]*mat[1][0], vect[1]*mat[1][1], vect[2]*mat[1][2] |
			//        		 |  vect[0]*mat[2][0], vect[1]*mat[2][1], vect[2]*mat[2][2] |
	
			Final Matrix 	         |  int_mat[0][0]+int_mat[0][1]+int_mat[0][2] |
			//         		 |  int_mat[1][0]+int_mat[1][1]+int_mat[1][2] |
			//        		 |  int_mat[2][0]+int_mat[2][1]+int_mat[2][2] |
	
			*/ 
	
			for (int i=0; i<n; i++) 
			{ 
				for(int j=0; j<n; j++)
				{
				vect_int[j][i]= vect_in_mult[i]*mat_in_mult[j][i]; // Fill intermediate matrix
				} 
			} 

			// Add All Elements Together
			for (int k=0; k<n; k++)
			{ 
			
				vect_out_mult[k] = vect_int[k][0]+vect_int[k][1]+vect_int[k][2]; 
		
			}	
			
		}
	
	

		// 3. Vector Subtraction 

		static void vector_subtract(double mat_1_sub[], double mat_2_sub[], double mat_result_sub[])

		{ 
			for(int i=0; i<n; i++)
			{

				mat_result_sub[i] = mat_1_sub[i]-mat_2_sub[i];  
	        
			}

		}

		// 4. Vector Addition 

		static void vector_add(double mat_1_add[], double mat_2_add[], double mat_result_add[])

		{ 
			for(int i=0; i<n; i++)
			{

				mat_result_add[i] = mat_1_add[i]+mat_2_add[i];  
	        
			}

		}

		// 5. Vector Cross Product 

		static void cross_product(double vect_1_cross[], double vect_2_cross[], double result_cross[])

		{ 

			// Result Validated Against TiNspire Result
			result_cross[0] = vect_1_cross[1]*vect_2_cross[2] - vect_1_cross[2]*vect_2_cross[1]; 
			result_cross[1] = vect_1_cross[2]*vect_2_cross[0] - vect_1_cross[0]*vect_2_cross[2]; 
			result_cross[2] = vect_1_cross[0]*vect_2_cross[1] - vect_1_cross[1]*vect_2_cross[0]; 
	
		}

		// 6. Matrix Inversion (3x3 Only - Haters Gonna Hate) 

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
}
