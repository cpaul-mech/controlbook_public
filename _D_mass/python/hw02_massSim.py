import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator #signal generator will not change for each HW!!
from massAnimation import massAnimation 
from dataPlotter import dataPlotter as dp #This is allowed, but not really necessary.

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1) #Reference is the thing we want our system to do.
zsignal = signalGenerator(amplitude=2.0*np.pi, frequency=0.1) # the theta will be the result of numerical integration of the position of the system.
rsignal = signalGenerator(amplitude=5, frequency=.5) #the tau comes from the controller.

# instantiate the simulation plots and animation
dataPlot = dp() #need to make them or we cannot use them. 
animation = massAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    z = zsignal.sin(t)
    F = rsignal.sawtooth(t)
    # update animation
    state = np.array([[z], [0.0]])  #state is made of theta, and theta_dot
    animation.update(state)
    dataPlot.update(t, r, state, F)
    # advance time by t_plot
    t = t + P.t_plot  
    plt.pause(0.001)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()