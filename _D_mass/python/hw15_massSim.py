#%% # Inverted Pendulum Parameter File
import massParam as P
from control import tf, bode
import matplotlib.pyplot as plt
#%%
# flag to define if using dB or absolute scale for M(omega)
dB_flag = True

# Compute inner and outer open-loop transfer functions
P_sys = tf([1/P.m],[1, P.b/P.m, P.k/P.m])


if __name__ == '__main__':
    # Plot the open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode(P_sys, dB=dB_flag)
    fig1.axes[0].set_title('$P_{in}(s)$')

    fig2 = plt.figure()
    bode(P_sys, dB=dB_flag)
    fig2.axes[0].set_title('$P_{out}(s)$')

    print('Close window to end program')
    plt.show()
