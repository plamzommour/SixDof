// This is the forward euler routine for one dimension
// Runge Kutta is also included but I have not gotten it to work. 

#include <bits/stdc++.h> // includes a lot of the standard libraries 
#include "integrate.h"

using namespace FE; 

void integrator::euler_integrate_6(double value_in_previous[], double value_in_current[], double dtime)
{ 
	for(int i=0;i<6;i++)
	{
	// 1-D euler integration 
	value_in_previous[i] += dtime*value_in_current[i]; 
	}
}

void integrator::euler_integrate_3(double value_in_previous[], double value_in_current[], double dtime)
{ 
	for(int i=0;i<3;i++)
	{
	// 1-D euler integration 
	value_in_previous[i] += dtime*value_in_current[i]; 
	}
}

void integrator::euler_integrate_4(double value_in_previous[], double value_in_current[], double dtime)
{ 
	for(int i=0;i<4;i++)
	{
	// 1-D euler integration 
	value_in_previous[i] += dtime*value_in_current[i]; 
	}
}

void integrator::rk4_integrate_6(double value_in_previous[], double value_in_current[], double dtime)
{ 

	double k1, k2, k3, k4; 
	
	for(int i=0;i<6;i++)
	{
	// RK4 Integration - 6 Values 
	
	k1 = dtime*value_in_current[i]; 
	k2 = dtime*(value_in_current[i]+k1/2); 
	k3 = dtime*(value_in_current[i]+k2/2); 
	k4 = dtime*(value_in_current[i]+k3); 
	
	value_in_previous[i] += ( (k1 + 2*k2 + 2*k3 + k4) / 6 );  
	
	}
}

void integrator::rk4_integrate_3(double value_in_previous[], double value_in_current[], double dtime)
{ 
	
	double k1, k2, k3, k4; 
	
	for(int i=0;i<3;i++)
	{
	// RK4 Integration - 3 Values  
	k1 = dtime*value_in_current[i]; 
	k2 = dtime*(value_in_current[i]+k1/2); 
	k3 = dtime*(value_in_current[i]+k2/2); 
	k4 = dtime*(value_in_current[i]+k3); 
	
	value_in_previous[i] += ( (k1 + 2*k2 + 2*k3 + k4) / 6 );  
	}
}

void integrator::rk4_integrate_4(double value_in_previous[], double value_in_current[], double dtime)
{ 
	
	double k1, k2, k3, k4; 
	
	for(int i=0;i<4;i++)
	{
	// RK4 Integration - 4 Values  
	k1 = dtime*value_in_current[i]; 
	k2 = dtime*(value_in_current[i]+k1/2); 
	k3 = dtime*(value_in_current[i]+k2/2); 
	k4 = dtime*(value_in_current[i]+k3); 
	
	value_in_previous[i] += ( (k1 + 2*k2 + 2*k3 + k4) / 6 );  
	}
}
