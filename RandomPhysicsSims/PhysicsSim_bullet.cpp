// Below is a simple simulation for a bullet. 

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
	
	// Air Density 
	const double rho = 1.225; 
	
	// Drag 
	const double cd = 0.295; 
	const double s = 4.80e-5; 
	
	// Thrust
	double thrust = 3619.00; 
	 
	// Gravity 
	const double grav = 9.81; 
	
	// Lift 
	const double cl = 0.05; // Magnus effect 
	double lift; 
	
	// Mass 
	const double mass = 0.015; // 15 grams in KG
	
	// Simulation Speed 
	const double dt = 0.001; // 1000 hz
	const double t_end = 15; 
	double time; 
	
	// Set up runtime variables
	double x_ddot = 17000; 
	double y_ddot = 17000; 
	double x_dot = 0; 
	double y_dot = 0; 
	double x = 0; 
	double y = 0; 
	double s_theta; 
	double c_theta; 
	double theta = (45 * M_PI/180); 
	double drag; 
	double airspeed; 
	
	// Open file
	std::ofstream file;
	file.open ("MyData.txt"); 
	
	std::cout<< "Running Simulation! Paul's Bullet Simulation! (^ - ^)  \n" << std::endl; 
	std::cout<< "Please See MyData.txt for Output \n" << std::endl; 
	
	// Print Data File header 
	file << "Time,x,y,x_dot,y_dot,theta" << std::endl; 
	
	// Enter runtime loop 
	while(time <= t_end)
	{
	
	// Find Trig Parameters 
	s_theta = sin(theta); 
	c_theta = cos(theta); 
	
	// Apply Thrust for 0.01 sec
	if (time >= 0.01)
	{ 
	thrust = 0.0; 
	}
	
	// Calculate airspeed
	airspeed = sqrt( (x_dot * x_dot) + (y_dot * y_dot) ); 
	
	// Calculate drag
	drag = 0.5 * rho * ( airspeed * airspeed ) * cd * s; 
	
	// Calculate lift
	lift = 0.5 * rho * ( airspeed * airspeed ) * cl * s; 
	
	// X-Direction Acceleration
	x_ddot = (1/mass) * ( (thrust * c_theta) - (lift * s_theta) - (drag * c_theta) ); 
	
	// Y-Direction Acceleration
	y_ddot = (1/mass) * ( (lift * c_theta) + (thrust * s_theta) - (drag * s_theta) - grav);
	
	// Integrate X Acceleration 2x
	x_dot = euler(x_dot, x_ddot, dt); 
	x = euler(x, x_dot, dt); 
	
	// Integrate Y Acceleration 2x 
	y_dot = euler(y_dot, y_ddot, dt); 
	y = euler(y, y_dot, dt); 
	
	// Calculate New Theta 
	theta = atan(y_dot/x_dot); 
	
	// Write to File 
	
	file << time << ","<< x << ","<< y << ","<< x_dot << "," << y_dot << ","<< theta << std::endl; 
	
	////// Check to see if you hit the ground or not 
	
	if (y < 0)
	{ 
	
		std::cout<< "You have hit the ground!  " << std::endl; 
		exit(0); 
		file.close(); 
	
	}
	
	// Iterate time
	time += dt; 
	
	} 
	
	// Output results to text file
	file.close(); 

	return 0; 
} 

