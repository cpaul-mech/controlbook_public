# Ball on Beam Parameter File
import numpy as np
# import control as cnt

# Physical parameters of the  ballbeam known to the controller
m1 =  0.35 # Mass of the block, kg
m2 =  2.0 # mass of beam, kg
length =  0.5 # length of beam, m
g =  9.81 # gravity at sea level, m/s^2

# parameters for animation
width = 0.05  # width of block
height = width*0.25  # height of block

# Initial Conditions
z0 = 0.0 # initial block position,m
theta0 = 0.0*np.pi/180  # initial beam angle,rads
zdot0 =  0.0 # initial speed of block along beam, m/s
thetadot0 = 0.0 # initial angular speed of the beam,rads/s

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end =  10.0 # End time of simulation
Ts = .01  # sample time for simulation
t_plot = 1.0/30.0  # the plotting and animation is updated at this rate

# saturation limits
Fmax = 15.0  # Max Force, N 
# I have no idea whether this is the correct value for Fmax.

# dirty derivative parameters
# sigma =   # cutoff freq for dirty derivative
# beta =  # dirty derivative gain

# equilibrium force when block is in center of beam
ze = length/2.0
Fe = m1*g*z0/length + m2*g/2.0
