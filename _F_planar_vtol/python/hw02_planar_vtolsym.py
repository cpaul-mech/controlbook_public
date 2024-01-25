import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from signalGenerator import signalGenerator #signal generator will not change for each HW!!
from VTOLAnimation import VTOLAnimation 
from dataPlotter import dataPlotter as dp #This is allowed, but not really necessary.

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1) #Reference is the thing we want our system to do.
zsignal = signalGenerator(amplitude=4.0, frequency=0.1,y_offset=5.0) # the theta will be the result of numerical integration of the position of the system.
rsignal = signalGenerator(amplitude=5, frequency=.5) #the tau comes from the controller.
thetasignal = signalGenerator(amplitude=np.pi/12, frequency=0.1) # the theta will be the result of numerical integration of the position of the system.
hsignal = signalGenerator(amplitude=4.0, frequency=0.5, y_offset=5.0) #Reference is the thing we want our system to do.
torque = 0.0
# instantiate the simulation plots and animation
dataPlot = dp() #need to make them or we cannot use them. 
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    z = zsignal.sin(t)
    F = rsignal.sawtooth(t)
    h = hsignal.sin(t)
    theta = thetasignal.sin(t)
    tau = torque
    # update animation
    state = np.array([[z], [h], [theta]])  #state is made of theta, and theta_dot
    animation.update(state)
    dataPlot.update(t, state,r, r, F, tau)
    # advance time by t_plot
    t = t + P.t_plot  
    plt.pause(0.001)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()