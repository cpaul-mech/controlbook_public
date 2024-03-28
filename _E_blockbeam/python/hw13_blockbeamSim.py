import matplotlib.pyplot as plt
import numpy as np
import blockbeamParam as P
from blockBeamDynamics import blockBeamDynamics
from ctrlObserver import ctrlObserver
from signalGenerator import signalGenerator
from blockbeamAnimation import blockbeamAnimation
from dataPlotter import dataPlotter
from dataPlotterObserver import dataPlotterObserver

# instantiate blockBeam, controller, and reference classes
blockBeam = blockBeamDynamics(alpha=0.0)
controller = ctrlObserver()
reference = signalGenerator(amplitude=0.05, frequency=0.1, y_offset=0.05)
disturbance = signalGenerator(amplitude=0.25, frequency=0.01)
noise_z = signalGenerator(amplitude=0.01)
noise_th = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = blockbeamAnimation()
plt.pause(5)  # 5 second pause
t = P.t_start  # time starts at t_start
y = blockBeam.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)  # reference input
        d = 0.0 # disturbance.step(t)  # input disturbance
        n = np.array([[0.0], [0.0]])  #noise.random(t)  # simulate sensor noise, will use in future assignments
        u, xhat = controller.update(r, y + n)  # update controller
        y = blockBeam.update(u + d)  # propagate system, "d" is a disturbance used in future assignments
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(blockBeam.state)
    dataPlot.update(t, r, blockBeam.state, u)
    dataPlotObserver.update(t, blockBeam.state, xhat, d, 0.0)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
