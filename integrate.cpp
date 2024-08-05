// This is the forward euler routine for one dimension

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
