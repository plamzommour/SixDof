/* 

This is the actual implementation of a Runge Kutta 4 Integration Routine

This takes in a single state and an associated timestep. 

The result is the single step numerical integral of the input

Like Euler - y_curr = y_prev + y'dt

*/

#include <cmath> 
#include "./integrate.h"

double AdamsBashforth(double state, double prev_state, double t_step)
{ 
	// Integrate and Output Resultant Step
	state += t_step * ( (1.5 * state) - (0.5 * prev_state) ); 
	return state ; 

} 
