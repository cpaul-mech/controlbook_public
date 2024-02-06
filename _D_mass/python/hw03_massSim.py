import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator #signal generator will not change for each HW!!
from massAnimation import massAnimation 
from massDynamics import massDynamics
from dataPlotter import dataPlotter as dp #This is allowed, but not really necessary.

# instantiate reference input classes
mass = massDynamics()
reference = signalGenerator(amplitude=0.5, frequency=0.02)
force = signalGenerator(amplitude=10.0, frequency=1.0)

# instantiate the simulation plots and animation
dataPlot = dp() #need to make them or we cannot use them. 
animation = massAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    t_next_plot = t + P.t_plot  # time to next plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)
        u = force.square(t)
        y = mass.update(u)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, r, mass.state, u)
    # advance time by t_plot
    plt.pause(0.001)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()