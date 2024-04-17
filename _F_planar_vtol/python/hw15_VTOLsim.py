#%% VTOL Parameter File
import VTOLParam as P
from control import tf, bode
import matplotlib.pyplot as plt
import sympy as sp
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
import numpy as np
#%%
# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# Compute inner and outer open-loop transfer functions
F_H = tf([1.0/(P.mc + 2.0*P.mr)],[1, 0, 0])
P_lon = F_H
#%%
Tau_theta = tf([(1.0/(P.Jc + 2.0*P.mr*P.d**2))],[1, 0, 0])
P_lat_in = Tau_theta
#%%
Theta_Z = tf([-P.Fe/(P.mc + 2.0*P.mr)],[1, (P.mu/(P.mc + 2.0*P.mr)), 0])
P_lat_out = Theta_Z

#%%
if __name__ == '__main__':
    print('F_H =', F_H)
    print('Tau_theta =', Tau_theta)
    print('Theta_Z =', Theta_Z)
    # Plot the open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode(F_H, dB=dB_flag)
    fig1.axes[0].set_title('$F(s)->H(s)$')

    fig2 = plt.figure()
    bode(Tau_theta, dB=dB_flag)
    fig2.axes[0].set_title('$Tau(s)->Theta(s)$')

    fig3 = plt.figure()
    bode(Theta_Z, dB=dB_flag)
    fig3.axes[0].set_title('$Theta(s)->Z(s)$')

    print('Close window to end program')
    plt.show()
