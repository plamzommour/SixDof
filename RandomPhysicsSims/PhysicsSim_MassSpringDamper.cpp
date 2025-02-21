// Below is a simple simulation for a Mass Spring Damper System 

// Use F = MA 

// 

#include <bits/stdc++.h> 

using namespace std; 

double euler(double x_prev, double x_dot, double dt)
{ 
	// Euler Algorithm => X_int = x_int + x_dot*dt; 
	return x_prev += x_dot*dt; 
}

int main() 
{ 
	
	// Set up constants below here 
	const double mass = 1; 
	const double damp = 0.25; 
	const double k_const = 2; 
	
	// Set up runtime variables
	double y_ddot = 0.00001; 
	double y_dot = 0.0001; 
	double y = 1; 
	
	// Simulation Parameters 
	const double dt = 0.001; // 1000 hz
	double time = 0; 
	const double t_end = 40; 
	
	// Open file
	std::ofstream file;
	file.open ("MyData.txt"); 
	
	std::cout<< "Running Simulation! Paul's Spring Simulation! (^ - ^)  \n" << std::endl; 
	std::cout<< "Please See MyData.txt for Output \n" << std::endl; 
	
	// Print Data File header 
	file << "Time,y,y_dot" << std::endl; 
	
	// Enter runtime loop 
	while(time <= t_end)
	{
	
	// Calculate Y double dot -- Dynamics Calculation -- Mx_ddot + Cx_dot + Kx = 0
	y_ddot = (1/mass) * ( -(damp * y_dot) - (k_const * y) ); 
	
	// Integrate Y Acceleration 2x 
	y_dot = euler(y_dot, y_ddot, dt); 
	y = euler(y, y_dot, dt); 
	
	// Write to File 
	
	file << time << ","<< y << "," << y_dot << std::endl; 
	
	time += dt; 
	
	} 
	
	// Output results to text file
	file.close(); 

	return 0; 
} 

