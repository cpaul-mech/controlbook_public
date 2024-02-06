import matplotlib.pyplot as plt
import numpy as np
import blockbeamParam as B
from signalGenerator import signalGenerator #signal generator will not change for each HW!!
from blockbeamAnimation import blockbeamAnimation 
from dataPlotter import dataPlotter as dp #This is allowed, but not really necessary.
from blockBeamDynamics import blockBeamDynamics

# instantiate reference input classes
blockbeam = blockBeamDynamics()
reference = signalGenerator(amplitude=0.25, frequency=0.2) #Reference is the thing we want our system to do.
force = signalGenerator(amplitude=.5, frequency=1.0, y_offset = 11.5)

# instantiate the simulation plots and animation
dataPlot = dp() #need to make them or we cannot use them. 
animation = blockbeamAnimation()

t = B.t_start  # time starts at t_start
while t < B.t_end:  # main simulation loop
    # set variables
    t_next_plot = t + B.t_plot  # time to next plot
    while t< t_next_plot:
        r = reference.square(t)
        u = force.sin(t)
        y = blockbeam.update(u)
        t = t + B.Ts
        # update animation
    animation.update(blockbeam.state)
    dataPlot.update(t, r, blockbeam.state, u)
    plt.pause(0.001)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()