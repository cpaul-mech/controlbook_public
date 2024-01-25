# VTOL Parameter File
import numpy as np

# Physical parameters of the  VTOL known to the controller
mc = 1.5 # kg
mr =  0.2 # kg
Jc =  1.1 # kg m^2
d = 0.05  # m
mu = 2.0  # kg/s
g = 9.81  # m/s^2
F_wind = 10 # wind disturbance force is zero in initial homeworks

# parameters for animation
length = 10.0

# Initial Conditions
z0 = 1.0  # initial lateral position
h0 =  1.0 # initial altitude
theta0 = np.pi*(1.0/16.0) # initial roll angle
zdot0 = 0  # initial lateral velocity
hdot0 = 0  # initial climb rate
thetadot0 = 0 # initial roll rate
target0 = 0

# Simulation Parameters
t_start = 0 # Start time of simulation
t_end =  10 # End time of simulation
Ts =  0.01 # sample time for simulation
t_plot = 1.0/30.0 # the plotting and animation is updated at this rate

# saturation limits
fmax = 10  # Max Force, N

# dirty derivative parameters
# sigma =   # cutoff freq for dirty derivative
# beta =  # dirty derivative gain

# equilibrium force
# Fe =

# mixing matrix
unmixing = np.array([[1.0, 1.0], [d, -d]]) # converts fl and fr (LR) to force and torque (FT)
mixing = np.linalg.inv(unmixing) # converts force and torque (FT) to fl and fr (LR) 

