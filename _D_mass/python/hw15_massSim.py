#%% # Inverted Pendulum Parameter File
import massParam as P
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
P_sys = tf([1/P.m],[1, P.b/P.m, P.k/P.m])
print('P_sys =', P_sys)
#%%
s = sp.symbols('s')
jw = sp.symbols('jw')
P_sys_handwork = (1/P.m)/(s**2 + P.b/P.m*s + P.k/P.m)
print('P_sys_handwork =')
display(Math(vlatex(P_sys_handwork)))
P_sys_handwork = P_sys_handwork.subs(s, jw)
print('P_sys_handwork =')
display(Math(vlatex(P_sys_handwork)))
k = 1/P.k
wn = sp.sqrt(P.k/P.m)
zeta = (P.b/P.m)*(1/np.sqrt(P.k/P.m))**2



#%%
if __name__ == '__main__':
    # Plot the open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode(P_sys, dB=dB_flag)
    fig1.axes[0].set_title('$P(s)$')

    print('Close window to end program')
    plt.show()
