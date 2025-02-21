// Below is a simple simulation for a bullet. 

// Use F = MA 

// 

#include <bits/stdc++.h> 

using namespace std; 

// Function for numerical integration - Simple euler's method.
double euler(double x_prev, double x_dot, double dt)
{ 
	// Euler Algorithm => X_int = x_int + x_dot*dt; 
	return x_prev += x_dot*dt; 
}

// Function to read inputs for simulation
void read_inputs(double& out)
{ 
	// Read input for coefficient of restitution 
	std::ifstream inputFile("input.txt");
        std::string key;
        
        // Read In Input 
        while (inputFile >> key) 
        {
        if (key == "c") inputFile >> out;
        }
}

int main() 
{ 
	
	// Set up constants below here 
	const double g = 9.81;  
	double c; 
	
	// Read From Input File 
	read_inputs(c);  
	
	// Set up runtime variables
	double y_dot = 0.0001; 
	double y = 10; 
	
	// Simulation Parameters 
	const double dt = 0.001; // 10000 hz
	double time = 0; 
	const double t_end = 40; 
	
	// Open file
	std::ofstream file;
	file.open ("MyData.txt"); 
	
	std::cout<< "Running Simulation! Paul's Bouncing Ball Simulation! (^ - ^)  \n" << std::endl; 
	std::cout<< "Please See MyData.txt for Output \n" << std::endl; 
	
	// Print Data File header 
	file << "Time,y,y_dot" << std::endl; 
	
	// Enter runtime loop 
	while(time <= t_end)
	{ 
	
	// Calculate Y_Dot 
	y_dot += -g * dt; 
	
	// Account for Ground Collision 
	if (y < 0)
	{ 
		y_dot = -c * y_dot; 
	}
	
	// Integrate Y Velocity 1x 
	y = euler(y, y_dot, dt); 
	
	// Write to File 
	
	file << time << ","<< y << "," << y_dot << std::endl; 
	
	time += dt; 
	
	} 
	
	// Output results to text file
	file.close(); 

	return 0; 
} 

