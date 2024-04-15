#%% satellite parameter file
import VTOLParam as P
from ctrlPID import ctrlPID
import hw15_VTOLsim as P15
from control import tf, bode
import matplotlib.pyplot as plt
import numpy as np
P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P15.dB_flag

# Compute inner and outer open-loop transfer functions
F_H = tf([1.0/(P.mc + 2.0*P.mr)],[1, 0, 0])
print('F_H =', F_H)
#%%
Tau_theta = tf([(1.0/(P.Jc + 2.0*P.mr*P.d**2))],[1, 0, 0])
print('Tau_theta =', Tau_theta)
#%%
Theta_Z = tf([-P.Fe/(P.mc + 2.0*P.mr)],[1, (P.mu/(P.mc + 2.0*P.mr)), 0])
print('Theta_Z =', Theta_Z)

# Compute controller transfer functions
# PD control xfer function for F_H loop
C_F_H = tf([(P10.kd_h+P10.kp_h*P10.sigma),
            (P10.kp_h+P10.ki_h*P10.sigma),
            P10.ki_h],
           [P10.sigma, 1, 0])
#%%
# PD control xfer function for inner loop
C_in = tf([(P10.kd_h+P10.sigma*P10.kp_h), P10.kp_h], [P10.sigma, 1])

# PID xfer function for outer loop
C_out = tf([(P10.kd_th+P10.kp_th*P10.sigma),
            (P10.kp_+P10.ki_z*P10.sigma),
            P10.ki_phi],
           [P10.sigma, 1, 0])
#%%
if __name__ == '__main__':

     omegas = np.logspace(-2, 3, 1000)

     # display bode plots of transfer functions
     fig1 = plt.figure()
     bode([F_H, F_H*C_F_H],
          omega=omegas, dB=dB_flag)
     plt.legend(['$P_{in}(s)$', '$C_{in}(s)P_{in}(s)$'])
     fig1.axes[0].set_title('VTOL, Lateral dynamics')
#%%
     fig2 = plt.figure()
     bode([P_out, P_out*C_out], omega=omegas, dB=dB_flag)
     plt.legend(['$P_{out}(s)$', '$C_{out}(s)P_{out}(s)$'])
     fig2.axes[0].set_title('Satellite, Outer Loop')

     print('Close window(s) to end program')
     plt.show()
