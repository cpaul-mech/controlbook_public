import matplotlib.pyplot as plt
import numpy as np
import rodMassParam as PE
from signalGenerator import signalGenerator
from rodMassAnimation import rodMassAnimation
from dataPlotter import dataPlotter
from rodMassDynamics import rodMassDynamics
from controllerPID import controllerPID

# instantiate system, controller, and reference classes
rodMass = rodMassDynamics()
controller = controllerPID()
reference = signalGenerator(amplitude=20*np.pi/180.0, frequency=0.1)
disturbance = signalGenerator(amplitude=0.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = rodMassAnimation()
plt.pause(3) # Allow time for the plot to open
t = PE.t_start
y = rodMass.h()
while t < PE.t_end:
    t_next_plot = t + PE.t_plot
    while t < t_next_plot:
        r = reference.square(t)
        d = disturbance.step(t)
        n = 0.0  #noise.random(t)
        u = controller.update(r, y)
        y = rodMass.update(u + d)
        t = t + PE.Ts
    # update animation and data plots
    animation.update(rodMass.state)
    dataPlot.update(t, r, rodMass.state, u)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
