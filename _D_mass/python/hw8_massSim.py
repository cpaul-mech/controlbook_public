import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from massDynamics import massDynamics
from signalGenerator import signalGenerator
from ctrlPD import ctrlPD
from massAnimation import massAnimation
from dataPlotter import dataPlotter

# instantiate satellite, controller, and reference classes
mass = massDynamics()
controller = ctrlPD()
reference = signalGenerator(amplitude=0.5, frequency=0.04)
disturbance = signalGenerator(amplitude=0.25)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()
t = P.t_start  # time starts at t_start
y = mass.h() # output of system at start of simulation
# begin main simulation loop:
while t < P.t_end:  # main simulation loop
    # propogate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        r = reference.square(t) # Reference input
        d = disturbance.step(t) # Disturbance input
        n = 0.0 #noise.random(t)  # Noise input
        x = mass.state
        u = controller.update(r, x) # Calculate the control value
        y = mass.update(u + d) # Propagate the dynamics
        t = t + P.Ts
    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, r, mass.state, u)
    plt.pause(0.01)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()