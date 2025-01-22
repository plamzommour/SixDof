// This is the Kinematics Main Driver

/* This is called after ONE cycle of the equations of motion. 

Knowns and Things to Do: 

	1. The output of the dynamics script are all in BODY frame 
	2. The EOM Outputs need to be rotated to the Local Level Frame 
	3. Euler Angles have to be extracted from the Local Level Quaternion
	4. The location of the local level frame origin is the current waypoint 
	5. The Local Level frame has to be rotated to the ECEF frame 
	6. The Current Latitude and Longitude has to be extracted from the ECEF Location 
	7. The ECEF Location has to be converted to the ECI Location 

*/  

#include <cmath>
#include "./kinematics_driver.h"
#include "./vector.h"
#include "../dynamics/integrate.h"
#include "./ll_kin.h"

void kinematics(comvar* s_data)
{ 
	
	// Find Local Level Kinematics Results and Pack Away
	ll_kin(s_data); 
	
	// Find Earth Kinematics Results and Pack Away
	
	
}
