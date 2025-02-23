# SixDof
Six Degree of Freedom Sim

Flat-Earth Sim: 

This sim runs the 6DoF equations of motion in eom.cpp using a driver script, driver.cpp. 

To build it, you need to run "build.sh".  This will work on a linux computer with gcc installed. 

For convenience, I have included DataReduction.m, which was created and will run on GNU octave or matlab. 

All kinematics are defined using quaternions, and support scripts/aero are also included. 

ECEF/Rotating Earth Sim:

A work-in-progress 6DoF sim that takes lessons learned during the flat earth sim development and applies them to a rotating earth sim. 

This sim is implementing enhanced class syntax for ease of use and a unified variable structure. 

Also included is an enhanced build script and a more organized directory architecture with dedicated inputs and outputs. 

ECEF Sim does not currently build and has not achieved first motion yet.  I suspect it would take at least a few solid days to get first motion. 

Random Physics Sims: 

Some small C++, Matlab and Python scripts I generated to practice for a modeling, simulation and analysis interview with Anduril. 
There are a number of dynamic and physics-based simulations in there.  All run and all build.  I use the included build.sh script to build these things, but you can do what you please. 

Usage and accesses to this simulation are controlled by Paul D. Zimmer, original creator.  
Private access for now until bugs are worked out. 
