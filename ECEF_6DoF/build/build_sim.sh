#!/bin/bash 

# This is the build script for the ECEF Sim 

# Set Paths 
SIM_DIR="../src"
BUILD_DIR="../run"
OUTPUT_BIN="run_ecef_sim"

# Compiler Flags

# Use g++ C++ Compiler 
CXX="g++"

# -Wall

#    Enables most common compiler warnings. This helps catch potential issues in the code.

#-g

#    Includes debugging information in the compiled binary. This is useful for debugging with tools like gdb.

# -pedantic

#    Enforces strict compliance with the C++ standard. It ensures that non-standard extensions generate warnings or errors.

#-O0

#    Disables compiler optimizations. This is useful during debugging because it preserves the original structure of the code, making 
#    it easier to step through with a debugger.

CXXFLAGS="-Wall -g -pedantic -O0" 
INCLUDES="-I $SIM_DIR" 

# Find All Files 
SOURCES=$(find "$SIM_DIR" -name "*.cpp")

# Compile
$CXX $CXXFLAGS $INCLUDES $SOURCES -o $BUILD_DIR/$OUTPUT_BIN 
