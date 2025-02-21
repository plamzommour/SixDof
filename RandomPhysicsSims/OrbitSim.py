# This is an attempt to simulate and model a 2D orbit of a stable low-ish earth orbit similar
# to Gemini after watching the following YouTube video:

#  https://www.youtube.com/watch?v=22OCPbfY5SE

#  I have the derivations on paper somewhere. They are also in the video

# You need math, pyplot, numpy, pyatmos, and scipy to run this.

import math
import matplotlib.pyplot as plt
import numpy as np
from pyatmos import expo
from scipy.integrate import solve_ivp

print('Running Simulation!')

# We need to define initial conditions for rocket stuff.
# We need a gravitational acceleration
g_const = 9.81  # m/s2

# We need a coefficient of drag
c_drag = 0.45  # Terrible assumption but it will be nonlinear by Re in reality

# We need a reference surface area for drag (Half sphere)
area_s = (math.pi / 4) * 3.0 ** 2

# We need an initial height - h in meters
h0 = 0

# We need an initial flight path angle, where vertical is defined as zero (in radians - tuned parameter)
psi0 = 1.8 * (math.pi / 180)

# We need an initial downrange angle (radians)
theta0 = 0

# We need an initial velocity (m/s)
v0 = 0

# Need a height of gravity turn start (tuned parameter)
h_turn_start = 1200  # m

# Need a sim time
t_max = 40000

# Need mass stackups for launch, stage 2 and stage 3
# Approximately gemini mass properties. Had to tune to make it to orbit
mass_prop_1 = 134000 - 4319  # kg
mass_prop_2 = 28400  # kg
mass_payload = 5000  # kg
mass_struct_1 = 4319 + 2301  # kg
mass_struct_2 = 2301  # kg
mass_total_start = mass_prop_1 + mass_prop_2 + mass_payload + mass_struct_1
mass_total_stage2 = mass_prop_2 + mass_payload + mass_struct_2

# Need thrust parameters and time of burn
t_burn1 = 156 # Seconds
coast_time = 180  # seconds
t_burn2 = 202  # Seconds
thrust_const_1 = 1900000  # N
thrust_const_2 = 440000  # N

# Mass flow rate parameters - assume constant for this sim. Just get it working.
m_dot_1 = mass_prop_1 / t_burn1  # kg/sec
m_dot_2 = mass_prop_2 / t_burn2  # kg/sec

# Need the Radius of the Earth
rad_earth = 6371000  # m

# Begin setting up integration calculation and Equations of Motion from Derivation

def EoM(t, y):
    v = y[0]
    psi = y[1]
    theta = y[2]
    h = y[3]
    # Set up atmosphere - use standard atmosphere parameters here
    expo_alt = expo(h/1000)
    rho = expo_alt.rho
    # find gravitational acceleration, which varies by altitude
    grav = g_const / ((1 + (h / rad_earth)) ** 2)
    # Find drag - Constant Cd
    drag = 0.5 * rho * v ** 2 * area_s * c_drag

    # All variable below have a time dependency
    # Mass props and thrust state machine:

    # Launch Stage (1)
    if t < t_burn1:
        m = mass_total_start - (m_dot_1 * t)  # Burn through the mass up to t
        T = thrust_const_1
    # Launch Stage (2)
    elif t < t_burn1 + t_burn2:
        m = mass_total_stage2 - (m_dot_2 * (t - t_burn1))
        T = thrust_const_2
    # Coast and see what happens
    elif t < t_burn1 + t_burn2 + coast_time:
        m = mass_total_stage2 - (m_dot_2 * t_burn2)  # You have burned all propellant for first two burns
        T = 0
    # Insertion Burn 15 seconds after coasting
    elif t < t_burn1 + t_burn2 + coast_time + 15:
        m = mass_total_stage2 - (m_dot_2 * (t - t_burn1 - coast_time))
        T = thrust_const_2
    # Let it ride it out
    else:
        m = mass_total_stage2 - mass_prop_2  # You have burned all the propellant you can
        T = 0

    # Set up equations of motion

    # Conditions before gravity turn

    if h <= h_turn_start:
        psi_dot = 0  # Inhibit turning
        v_dot = T / m - drag / m - grav # Vertical flight, no flight path angle consideration
        theta_dot = 0 + 7.272e-5  # no downrange
        h_dot = v  # Altitude rate aligned with rocket velocity (vertical launch)

    # Conditions after gravity turn
    else:
        phi_dot = grav * np.sin(psi) / v  # Angle of flight with consideration to launch location vertical
        v_dot = T / m - drag / m - grav * np.cos(psi)  # Acceleration
        h_dot = v * np.cos(psi)  # Altitude rate
        theta_dot = ((v * (np.sin(psi))) / (rad_earth + h)) + 7.272e-5  # Downrange distance rate
        psi_dot = phi_dot - theta_dot  # Flight path angle

    # Define array with size for python's stupid specs
    dot_array = np.array([v_dot, psi_dot, theta_dot, h_dot], dtype="object")

    return dot_array

# Run Numerical Integration for Equations of Motion
sol = solve_ivp(EoM, [0, t_max], [v0, psi0, theta0, h0], max_step=1)

# Values coming out of solution are in meters, so need to convert to KM
v_plot = sol.y[0] / 1000  # km/sec
psi_plot = sol.y[1] * (180/math.pi)  # deg
theta_plot = sol.y[2] * (180/math.pi)  # deg
h_plot = sol.y[3] / 1000  # km
t_plot = sol.t
total_h = h_plot + rad_earth/1000 # for Polar Plotting
earth_plot = total_h - h_plot  # for earth plot

# Plot Everything

figure, ax = plt.subplots(2,2)
# Plot velocity

ax[0,0].plot(t_plot,v_plot)
ax[0,0].set_title('Velocity vs. Time')
ax[0,0].grid()
# plt.show()

# Plot altitude
ax[0,1].plot(t_plot, h_plot)
ax[0,1].set_title('Altitude vs Time')
ax[0,1].grid()
# plt.show()

# Plot Mass
ax[1,0].plot(t_plot, psi_plot)
ax[1,0].set_title('Flight Path Angle vs Time')
ax[1,0].grid()
# plt.show()

#  Plot Theta in Cartesian and Polar
ax[1,1].plot(t_plot, theta_plot)
ax[1,1].set_title('Downrange Distance (in Deg) vs. Time')
ax[1,1].grid()
plt.show()

plt.polar(theta_plot * (math.pi/180), total_h)
plt.polar(theta_plot * (math.pi/180), earth_plot)
plt.suptitle('Polar Plot of Orbit Above Earth')
plt.show()