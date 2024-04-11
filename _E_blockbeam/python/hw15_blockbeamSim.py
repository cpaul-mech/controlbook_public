#%% # Inverted Pendulum Parameter File
import blockbeamParam as P
from control import tf, bode
import matplotlib.pyplot as plt
import sympy as sp
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
import numpy as np
# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

#%%

# Compute inner and outer open-loop transfer functions
P_sys = tf([P.ell/(P.m2*P.ell**2/3 + P.m1*P.ze**2)],[1, 0, 0])
print('P_sys =', P_sys)


#%%
if __name__ == '__main__':
    # Plot the open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode(P_sys, dB=dB_flag)
    fig1.axes[0].set_title('$P(s)$')

    print('Close window to end program')
    plt.show()

# %%
