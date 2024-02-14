import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter
from hummingbirdDynamics import HummingbirdDynamics
from ctrlEquilibrium import ctrlEquilibrium
# instantiate reference input classes
phi_ref = SignalGenerator(amplitude=1.5, frequency=0.05)
theta_ref = SignalGenerator(amplitude=0.5, frequency=0.05)
psi_ref = SignalGenerator(amplitude=0.5, frequency=.05)
force_ell_ref = SignalGenerator(amplitude=1.0, frequency=.05)
force_r_ref = SignalGenerator(amplitude=1.0, frequency=.03)

# instantiate the simulation plots and animation
dataPlot = DataPlotter()
animation = HummingbirdAnimation()
dynamics = HummingbirdDynamics()
ctrlr = ctrlEquilibrium()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    t_next_plot = t + P.t_plot  # set the next time to plot
    # set variables
    while t< t_next_plot:
        state = dynamics.state
        force_pwm = ctrlr.update(state)
        
        ref = np.array([[0.0], [0.0], [0.0]])
        dynamics.update(force_pwm)
        t = t + P.Ts
        
    animation.update(t, state)
    dataPlot.update(t, state, ref, force_pwm)
    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.05)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
