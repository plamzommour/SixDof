/* This is a script that runs the us standard atmosphere for 1 cycle

US Standard Atmosphere 1976 Standard for Temperature and Pressure.

Taken from Wikipedia.  

Script takes in Altitude and Velocity in Inertial Axes
and outputs dynamic pressure, static pressure and temperature

*/ 

// Draw Standard Atmosphere -- Up to 11 km

#include "atmosphere.h" 
#include <bits/stdc++.h>

using namespace AT; 

// Atmosphere - Only Good Up to 11km 
void atmos::run_1976_atmos(double altitude_m, double& static_pressure, double& temperature, double& density)
{ 

	double low_alt = 0; 
	double high_alt = 11000; // meters 
	double low_alt_press = 101325; //n/m2
	double high_alt_press = 22632.1; //n/m2 
	double low_alt_temp = 288.15; //k
	double high_alt_temp = 216.65; //k 
	
	// Negate Altitude - NED to actual Altitude 
	altitude_m = -altitude_m; 
	
	// Perform Interpolation on Static Pressure 
	// X Axis = Altitude 
	// Y Axis = Pressure in Pascals (n/m2)
	atmos::interp(low_alt, high_alt, low_alt_press, high_alt_press, altitude_m, static_pressure); 
	
	// Perform Interpolation on Temperature 
	// X Axis = Altitude 
	// Y Axis = Temperature in K
	atmos::interp(low_alt, high_alt, low_alt_temp, high_alt_temp, altitude_m, temperature);
	
	// Density - see apps.DTIC.mil/sti/tr/pdf/ADA588839.pdf
	// Output is kg/m3
	density = (static_pressure * 0.00348367635597379)/temperature; 
	
	
}

// 1-D Interpolation Routine 
void atmos::interp(double low_x, double high_x, double low_y, double high_y, double test_x, double& result_y)
{
	
	// Run 1-D Linear Interpolation 
	result_y = low_y + ( (high_y - low_y) / (high_x - low_x) ) * (test_x - low_x); 

} 

// Dynamic Pressure Calculation - MUY IMPORTANTE
// Takes in velocity as a vector and finds airspeed 
void atmos::q_bar(double airspeed[], double rho, double& q_out)
{ 
	
	// Dynamic Pressure
	q_out = 0.5 * rho * (airspeed[2] * airspeed[2]); 
	return; 
}

