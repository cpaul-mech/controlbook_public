# mass-spring-damper Parameter File
import numpy as np

# Physical parameters of the mass known to the controller
m = 5.0  # mass kg
k = 3.0  # spring constant Kg/s^2
b = 0.5  # damping coefficient Kg/s

# parameters for animation
length = 5.0
width = 1.0

# Initial Conditions
z0 = 0.0     # initial position of mass, m
zdot0 = 0.0  # initial velocity of mass m/s

# Simulation Parameters
t_start = 0.0 # Start time of simulation
t_end = 45.  # End time of simulation
Ts = 0.05   # sample time for simulation
t_plot = 1.0/30.0  # the plotting and animation is updated at this rate
# this is the step at which we're numericcally integrating, and the 30 
# is the rate of the sampling? might be an incorrect assumption.

# dirty derivative parameters
sigma = 0.05 # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

# saturation limits
F_max =  6.0 # Max force, N

