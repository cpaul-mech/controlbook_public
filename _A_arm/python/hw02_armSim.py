import matplotlib.pyplot as plt
import numpy as np
import armParam as P
from signalGenerator import signalGenerator #signal generator will not change for each HW!!
from armAnimation import armAnimation #might change on our HW
from dataPlotter import dataPlotter #might change on our HW.

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
thetaRef = signalGenerator(amplitude=2.0*np.pi, frequency=0.1)
tauRef = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = armAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    theta = thetaRef.sin(t)
    tau = tauRef.sawtooth(t)
    # update animation
    state = np.array([[theta], [0.0]])  #state is made of theta, and theta_dot
    animation.update(state)
    dataPlot.update(t, r, state, tau)
    # advance time by t_plot
    t = t + P.t_plot  
    plt.pause(0.001)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
