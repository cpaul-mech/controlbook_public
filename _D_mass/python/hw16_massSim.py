#%% satellite parameter file
import massParam as P
from ctrlPID import ctrlPID
import hw15_massSim as P15
from control import tf, bode
import control as ctrl
import matplotlib.pyplot as plt
import numpy as np
P10 = ctrlPID()
# flag to define if using dB or absolute scale for M(omega)
dB_flag = P15.dB_flag

#%%
# Compute inner and outer open-loop transfer functions
P_sys = tf([1/P.m],[1, P.b/P.m, P.k/P.m])
print('P_sys =', P_sys)

# Compute controller transfer functions

# PID xfer function for outer loop
C_pid = tf([(P10.kd+P10.kp*P10.sigma),
            (P10.kp+P10.ki*P10.sigma),
            P10.ki],
           [P10.sigma, 1, 0])

#%%
frequencies = [10**-2, 10**-1]

# Use the control.freqresp() function to return the values at the specified frequencies
mags, phases, omegas = ctrl.freqresp(P_sys*C_pid, frequencies)

# Print the values
print(mags)
# Calculate the slope of the magnitude plot with the specified freq and mag values
slope = (mags[1] - mags[0]) / frequencies[1] - frequencies[0]
system_type = slope
print(system_type)

#%%
if __name__ == '__main__':

    omegas = np.logspace(-2, 3, 1000)

    # display bode plots of transfer functions
    fig1 = plt.figure()
    bode([P_sys, P_sys*C_pid],
         omega=omegas, dB=dB_flag)
    plt.legend(['$P(s)$', '$C(s)P(s)$']) #'$\\frac{1}{s^2}$'])
    fig1.axes[0].set_title('Satellite, Inner Loop')
    plt.show()

# %%
