/* This is the header file for a Runge Kutta 4 (RK4) Integration Routine. 

Easy peasy! 

*/ 

#include <cmath> 

// The following is protection from too many definitions
#ifndef INTEGRATE_H 
#define INTEGRATE_H

// We must declare the RK4 Integrator Function 

double AdamsBashforth(double state, double prev_state, double t_step); 

#endif // INTEGRATE_H
