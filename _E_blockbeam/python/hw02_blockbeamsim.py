import matplotlib.pyplot as plt
import numpy as np
import blockbeamParam as B
from signalGenerator import signalGenerator #signal generator will not change for each HW!!
from blockbeamAnimation import blockbeamAnimation 
from dataPlotter import dataPlotter as dp #This is allowed, but not really necessary.

# instantiate reference input classes
reference = signalGenerator(amplitude=0.25, frequency=0.2) #Reference is the thing we want our system to do.
zsignal = signalGenerator(amplitude=0.25, frequency=0.7,y_offset=0.25) #
rsignal = signalGenerator(amplitude=5, frequency=.5) #the tau comes from the controller.
thetasignal = signalGenerator(amplitude=np.pi/5, frequency=0.1) # the theta will be the result of numerical integration of the position of the system.

# instantiate the simulation plots and animation
dataPlot = dp() #need to make them or we cannot use them. 
animation = blockbeamAnimation()

t = B.t_start  # time starts at t_start
while t < B.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    z = zsignal.sin(t)
    F = rsignal.sawtooth(t)
    theta = thetasignal.sin(t)
    # update animation
    state = np.array([[z], [theta]])  #state is made of z position and theta in 
    animation.update(state)
    dataPlot.update(t, r, state, F)
    # advance time by t_plot
    t = t + B.t_plot  
    plt.pause(0.001)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()