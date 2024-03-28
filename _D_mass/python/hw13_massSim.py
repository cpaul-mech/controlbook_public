import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from massDynamics import massDynamics
from ctrlStateFeedbackIntegrator import ctrlStateFeedbackIntegrator
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from dataPlotterObserver import dataPlotterObserver

# instantiate satellite, controller, and reference classes
mass = massDynamics(alpha=0.2)
controller = ctrlStateFeedbackIntegrator()
reference = signalGenerator(amplitude=0.5, frequency=0.1)
disturbance = signalGenerator(amplitude=0.25)
dataPlotObserver = dataPlotterObserver()

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()
plt.pause(4)
t = P.t_start  # time starts at t_start
y = mass.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)  # reference input
        d = disturbance.step(t)  # input disturbance
        n = np.array([[0.0],[0.0]])  #noise.random(t)  # simulate sensor noise, will use in future assignments
        x = mass.state
        u, xhat = controller.update(r, y+n)  # update controller
        y = mass.update(u + d)  # propagate system, "d" is a disturbance used in future assignments
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, r, mass.state, u)
    dataPlotObserver.update(t, x, xhat)
    plt.pause(0.001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
