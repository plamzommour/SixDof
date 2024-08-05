# !/bin/bash 

# Build Script for 6DoF
g++ -Wall -g -pedantic -o run_6dof matrix_operations.h quaternion_operations.cpp quaternion_operations.h eom.h eom.cpp integrate.h integrate.cpp driver.cpp atmosphere.h atmosphere.cpp aero.h aero.cpp


