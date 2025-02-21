// Header file for a class called "Kinematics" 

#include <bits/stdc++.h> 

class kinematics
{ 

	public: 
	
	static void run_kin(double data_in[], double time, double& x, double& y)
	{ 
	
	const double g = 9.81; 
	
	// v_init = data_in[0], theta = data_in[1], 
	x = data_in[0] * (cos(data_in[1] * (M_PI/180))) * time; 
	y = ( data_in[0] * (sin(data_in[1] * (M_PI/180))) * time ) - (0.5 * g * time * time); 
	
	} 


}; 
