# This is the python interpretation of the bullet sim I wrote in c++

import math
import matplotlib.pyplot as plt
import numpy as np
import sys

# Constants

rho = 1.225
cd = 0.295
s = 4.80e-5
thrust = 2900.00
grav = 9.81
cl = 0.05
mass = 0.015
dt = 0.001
t_end = 15
sim_time = 0
x_ddot = 0
y_ddot = 0
x_dot = 0
y_dot = 0
x = 0
y = 0
theta = (45 * math.pi / 180)
y_plot = []
x_plot = []
theta_plot = []
time_plot = []

# Enter Runtime Loop
while sim_time <= t_end:

    s_theta = math.sin(theta)
    c_theta = math. cos(theta)

    if sim_time >= 0.01:
        thrust = 0.0

    # Calculate Airspeed
    airspeed = math.sqrt((x_dot * x_dot) + (y_dot * y_dot))

    # Calculate Drag
    drag = 0.5 * rho * ( airspeed * airspeed ) * cd * s

    # Calculate lift
    lift = 0.5 * rho * ( airspeed * airspeed ) * cl * s

    # X-Direction Acceleration
    x_ddot = (1 / mass) * ( (thrust * c_theta) - (lift * s_theta) - (drag * c_theta) )

    # Y-Direction Acceleration
    y_ddot = (1 / mass) * ( (lift * c_theta) + (thrust * s_theta) - (drag * s_theta) - grav)

    # Integrate X Acceleration 2 Times
    x_dot = x_dot + x_ddot * dt
    x = x + x_dot * dt
    x_plot.append(x)

    # Integrate Y Acceleration 2 Times
    y_dot = y_dot + y_ddot * dt
    y = y + y_dot * dt
    y_plot.append(y)

    # Calculate New Theta
    theta = math.atan(y_dot / x_dot)
    theta_plot.append(theta * (180/math.pi) )
    time_plot.append(sim_time)

    if y < 0:
        print("You Have Hit the Ground")

        # Plot Everything

        fig, (ax1, ax2) = plt.subplots(2, 1)  # 2 rows, 1 column

        ax1.plot(x_plot, y_plot)
        ax1.set_title('Bullet Trajectory')
        ax1.set_xlabel('X Distance [meters]')
        ax1.set_ylabel('Y Distance [meters]')
        ax1.grid()

        # Plot altitude
        ax2.plot(time_plot, theta_plot)
        ax2.set_title('Theta vs Time')
        ax2.set_xlabel('Simulation Time [sec]')
        ax2.set_ylabel('Pitch Angle [deg]')
        ax2.grid()

        # Adjust layout to prevent overlapping
        plt.tight_layout()

        plt.show()

        sys.exit(0)  # Exit the program with success code

    sim_time = sim_time + dt

