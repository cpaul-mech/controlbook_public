import matplotlib.pyplot as plt
import blockbeamParam as P
import numpy as np
from blockBeamDynamics import blockBeamDynamics
from ctrlPD import ctrlPD
from signalGenerator import signalGenerator
from blockbeamAnimation import blockbeamAnimation
from dataPlotter import dataPlotter

# instantiate blockbeam, controller, and reference classes
blockbeam = blockBeamDynamics()
controller = ctrlPD()

# HW asked for reference input frequency of 0.01 Hz
reference = signalGenerator(amplitude=0.5, frequency=0.01)
disturbance = signalGenerator(amplitude=0.0, frequency=0.0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = blockbeamAnimation()
t = P.t_start  # time starts at t_start
y = blockbeam.h()  # output of the system at the start of the simulation
while t < P.t_end:  # main simulation loop
    
    # propogate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        r = reference.square(t) # reference input
        d = disturbance.step(t) # input disturbance
        n = 0.0 #noise.random(t)  # simulate sensor noise
        x = blockbeam.state
        u = controller.update(r, x)  # update controller
        y = blockbeam.update(u + d) # Propagate system
        t = t + P.Ts
    # update animation of the blockbeam
    animation.update(blockbeam.state)
    dataPlot.update(t, r, blockbeam.state, u)
    plt.pause(0.001) # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
        