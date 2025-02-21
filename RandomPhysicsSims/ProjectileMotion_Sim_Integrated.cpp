// Below is a simple simulation for a Projectile. 

// Use Kinematics formulas 

// 

#include <bits/stdc++.h> 

// Function to read inputs for simulation
void read_inputs(double out[])
{ 
	// Read input for coefficient of restitution 
	std::ifstream inputFile("input_kin.txt");
        std::string key;
        
        // Read In Input Line by Line
        while (inputFile >> key) 
        {
        if (key == "v_init") inputFile >> out[0];
        if (key == "theta") inputFile >> out[1]; 
        }
}

int main() 
{ 
	
	// Set up constants below here 
	const double g = 9.81;  
	double v_init, theta; 
	double c[2]; 
	double x, y, v_x, v_y; 
	
	// Read From Input File 
	read_inputs(c);  
	v_init = c[0]; 
	theta = c[1]; 
	
	// Simulation Parameters 
	const double dt = 0.001; // 10000 hz
	double time = 0; 
	const double t_end = 50; 
	
	// Open file
	std::ofstream file;
	file.open ("MyData.csv"); 
	
	std::cout<< "Running Simulation! Paul's Projectile Motion Simulation! (^ - ^)  \n" << std::endl; 
	std::cout<< "Please See MyData.csv for Output \n" << std::endl; 
	
	// Print Data File header 
	file << "Time,x,y" << std::endl; 
	
	// Initial Values 
	v_x = v_init * (cos(theta * (M_PI/180)));
	v_y = v_init * (sin(theta * (M_PI/180))); 
	x = 0; 
	y = 0; 
	
	// Enter runtime loop 
	while(time <= t_end)
	{ 
	
	// Calulate Kinematic Parameters for Velocity and Position Using Numerical Integration
	v_x = v_x;
	v_y -= g*dt; 
	x += v_x*dt; 
	y += v_y*dt;  
	
	////// Check to see if you hit the ground or not 
	
	if (y < 0)
	{ 
	
		std::cout<< "You have hit the ground!  " << std::endl; 
		exit(0); 
		file.close(); 
	
	}
	
	// Write to File 
	
	file << time << ","<< x << "," << y << std::endl; 
	
	time += dt; 
	
	} 
	
	// Output results to text file
	file.close(); 

	return 0; 
} 

