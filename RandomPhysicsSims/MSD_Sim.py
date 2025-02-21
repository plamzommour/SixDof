import math
import matplotlib.pyplot as plt
import numpy as np
import sys

# This is a python interpretation of the mass-spring-damper system I did in C++

# Constants
mass = 1
damp = 0.25
k_const = 2

# Runtime Variables
y_ddot = 0.00001
y_dot = 0.0001
y = 1

# Simulation Parameters
dt = 0.001
time = 0
t_end = 40
t_plot = []
y_plot = []

while time <= t_end:

    # Calculate y double dot
    y_ddot = (1 / mass) * (-(damp * y_dot) - (k_const * y))

    # Integrate Y Acceleration 2 Times
    y_dot = y_dot + y_ddot * dt
    y = y + y_dot * dt
    y_plot.append(y)
    t_plot.append(time)

    time += dt

# Plot Everything

plt.figure(figsize=(8, 5))
plt.plot(t_plot, y_plot, color='blue')
plt.title('Plot of Mass Spring Damper Trajectory')
plt.xlabel('Simulation Time')
plt.ylabel('Y Position [m]')
plt.grid(True)
plt.show()

"""
Python Statistics Functions Reference

# Built-in statistics module:
import statistics
statistics.mean(data)          # Arithmetic mean
statistics.median(data)        # Median
statistics.mode(data)          # Mode
statistics.stdev(data)         # Standard deviation
statistics.variance(data)      # Variance
statistics.median_low(data)    # Low median of data
statistics.median_high(data)   # High median of data
statistics.harmonic_mean(data) # Harmonic mean
statistics.geometric_mean(data)# Geometric mean

# Numpy library functions:
import numpy as np
np.mean(data)                  # Mean
np.median(data)                # Median
np.std(data)                   # Standard deviation
np.var(data)                   # Variance
np.percentile(data, q)         # Percentiles
np.corrcoef(data1, data2)      # Correlation coefficient
np.histogram(data)             # Histogram
np.quantile(data, q)           # Quantile

# Scipy.stats library functions:
from scipy import stats
stats.describe(data)           # Summary statistics
stats.ttest_1samp(data, popmean)  # One-sample t-test
stats.ttest_ind(data1, data2)     # Independent t-test
stats.norm()                   # Normal distribution functions
stats.skew(data)               # Skewness
stats.kurtosis(data)           # Kurtosis
"""

"""
How to Import Data from a .txt File in Python

# Using built-in open() and readlines():
with open('file.txt', 'r') as file:
    data = file.readlines()         # Reads all lines into a list

# Using open() and read():
with open('file.txt', 'r') as file:
    data = file.read()              # Reads the entire file as a single string

# Reading line by line:
with open('file.txt', 'r') as file:
    for line in file:
        print(line.strip())         # Process each line individually

# Using numpy:
import numpy as np
data = np.loadtxt('file.txt')       # Loads numerical data into a NumPy array
data = np.genfromtxt('file.txt', delimiter=',')  # For CSV or custom delimiters

"""