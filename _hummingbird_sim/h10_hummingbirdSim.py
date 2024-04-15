import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter
from hummingbirdDynamics import HummingbirdDynamics
from ctrlPID import ctrlPID

# instantiate pendulum, controller, and reference classes
hummingbird = HummingbirdDynamics(alpha=0.2)
controller = ctrlPID()
psi_ref = SignalGenerator(amplitude=15.*np.pi/180., frequency=1.0/5.0)
theta_ref = SignalGenerator(amplitude=15.*np.pi/180., frequency=1.0/5.0)

# instantiate the simulation plots and animation
dataPlot = DataPlotter()
animation = HummingbirdAnimation()
plt.pause(3)  # Allow time for the plot to open
t = P.t_start  # time starts at t_start
y = hummingbird.h()
while t < P.t_end:  # main simulation loop
    # Propagate dynamics at rate Ts
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        r = np.array([[0.0], [psi_ref.square(t)]]) #r = [[theta_ref], [psi_ref]]
        u, y_ref = controller.update(r, y)
        y = hummingbird.update(u)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots at rate t_plot
    animation.update(t, hummingbird.state)
    dataPlot.update(t, hummingbird.state, y_ref, u)

    # the pause causes figure to be displayed during simulation
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
