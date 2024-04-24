import numpy as np
import control as cnt

# physical parameters
m = 0.1
ell = 0.25
b = 0.1
g = 9.8
k1 = 0.02
k2 = 0.01
tau_max = 3

# simulation parameters
t_start = 0.0
t_end = 100.0
Ts = 0.01
t_plot = 0.1

# equilibrium force
theta_e = 0.0
tau_eq = ell*g*m*np.cos(theta_e) + 0.5*k1*theta_e + k2*theta_e**3

