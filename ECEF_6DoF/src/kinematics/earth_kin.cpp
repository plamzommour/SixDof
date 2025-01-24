// This is the geodesy placeholder.  This handles all the Earth-Related Stuff

// Origin of Local Level frame is assumed to be Current Waypoint

// NED to ECEF, ECEF to Geodetic, ECEF to ECI

#include <cmath>
#include "./earth_kin.h"
#include "./vector.h"

void earth_kin(comvar* s_data)
{

	// Create temp variables 
	
	double q_ll_2_ecef_int [4]; 
	double q_ecef_2_ll_int [4]; 
	double q_ll_2_ecef_temp [4]; 
	double q_ecef_2_ll_temp [4]; 
	double n_ecef; 
	
	// Extract Current Waypoint Data and Set the Quaternion and Local Level Origin to That // 
	
	// Local Level to ECEF
	Quaternion q_rot1_ecef_2_ll (cos(s_data->wp_longitude/2), 0, 0, sin(s_data->wp_longitude/2)); // Z Axis About Longitude 
	Quaternion q_rot2_ecef_2_ll (cos(s_data->wp_latitude/2), 0, sin(s_data->wp_latitude/2), 0); // Y Axis About Latitude 
	Quaternion q_ecef_2_ll_result; 
	
	// Construct Rotation Quaternion and Normalize it - ECEF to Local Level
	// 3-2 Rotation Sequence 
	q_ecef_2_ll_result = q_rot1_ecef_2_ll.quat_mult(q_rot2_ecef_2_ll); 
	q_ecef_2_ll_result.normalize();
	s_data->q_ecef_2_ll = q_ecef_2_ll_result; 
	s_data->q_ll_2_ecef = q_ecef_2_ll_result.conjugate(); 
	
	// Calculate Current Waypoint ECEF Coordinates 
	Vector ll_ecef_loc; 
		
		// 1. Find N parameter
		n_ecef = s_data->a / ( sqrt(1 - ( (s_data->e*s_data->e) * 
					( (sin(s_data->wp_latitude)) * (sin(s_data->wp_latitude)) ) ) ) ); 
					
		// 2. Find ECEF Location of Waypoint 
		Vector wp_loc_ecef; 
		wp_loc_ecef.x = (n_ecef + s_data->wp_alt)*cos(s_data->wp_latitude)*cos(s_data->wp_longitude);
		wp_loc_ecef.y = (n_ecef + s_data->wp_alt)*cos(s_data->wp_latitude)*sin(s_data->wp_longitude);
		wp_loc_ecef.z = ( ( n_ecef * (1 - (s_data->e*s_data->e) ) ) + s_data->wp_alt)*cos(s_data->wp_latitude)*
				sin(s_data->wp_latitude);
	
	
	// Rotate Current Local Level Coordinates to ECEF Orientation and Add them to the Current ECEF Origin Location
	Vector ll_pos; 
	Vector pos_ecef_temp; 
	ll_pos.x = s_data->x_ll; 
	ll_pos.y = s_data->y_ll; 
	ll_pos.z = s_data->z_ll; 
	
	// Rotate Local Level Position to ECEF Orientation 
	pos_ecef_temp = s_data->q_ll_2_ecef.rotate(ll_pos); 
	
	// Sum LL Location in ECEF to ECEF Position of LL Origin to find Vehicle Position in ECEF
	Vector current_ecef_loc; 
	current_ecef_loc = pos_ecef_temp.add_vector(wp_loc_ecef); 
	
	// Pack Result Back Into Variables
	s_data->x_ecef = current_ecef_loc.x; 
	s_data->y_ecef = current_ecef_loc.y; 
	s_data->z_ecef = current_ecef_loc.z; 
	
	// Find Current Vehicle Latitude and Longitude -- Script from Chat GPT (experimental)

		// Constants for WGS84 ellipsoid
		const double a = 6378137.0;           // semi-major axis in meters
		const double e2 = 0.00669437999014;   // square of eccentricity

    		double x = s_data->x_ecef;
    		double y = s_data->y_ecef;
    		double z = s_data->z_ecef;
    		double latitude; 
    
    		// Calculate longitude
    		s_data->longitude = atan2(y, x);
    
   		// Iterate to calculate latitude and altitude using Newton-Raphson method
    		double p = sqrt(x * x + y * y);   // distance from z-axis
    		double theta = atan2(z * a, p * (1 - e2));  // latitude (initial guess)
    
    		// Iterate to refine latitude and altitude
    		const int maxIter = 100;
    		const double tolerance = 1e-9;
    		double sinTheta, cosTheta, N;
    		for (int i = 0; i < maxIter; ++i) {
        	sinTheta = sin(theta);
        	cosTheta = cos(theta);
        
        	N = a / sqrt(1 - e2 * sinTheta * sinTheta);  // prime vertical radius of curvature
        	double h = p / cosTheta - N;  // altitude
        	latitude = atan2(z + e2 * N * sinTheta, p);  // updated latitude
        
        	// If the change is small enough, break the loop
        	if (fabs(latitude - theta) < tolerance) {
            	break;
        	}
        
        	theta = latitude;
    		}
    
    		s_data->latitude = latitude; 
    
    		// Calculate altitude
    		s_data->altitude = p / cos(latitude) - N;

}



