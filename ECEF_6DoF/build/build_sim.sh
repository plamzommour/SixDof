#!/bin/bash 

# This is the build script for the ECEF Sim 

# Set Paths 
SIM_DIR="../src"
BUILD_DIR="."
OUTPUT_BIN="run_ecef_sim"

# Compiler 
CXX="g++"
CXXFLAGS="-Wall -g -pedantic -O0" 
INCLUDES="-I $SIM_DIR" 

# Find All Files 
SOURCES=$(find "$SIM_DIR" -name "*.cpp")

# Compile
$CXX $CXXFLAGS $INCLUDES $SOURCES -o $BUILD_DIR/$OUTPUT_BIN 
