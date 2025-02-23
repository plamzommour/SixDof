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

Also included is an enhanced build script and a more organized directory architecture. 

Usage and accesses to this simulation are controlled by Paul D. Zimmer, original creator.  
Private access for now until bugs are worked out. 
