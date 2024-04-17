#%% [markdown]
## Homework 18: Finding the C(s) for the system that meet the specifications
# Import the libraries and starting transfer functions
from control import bode, tf, margin, mag2db, step_response, tf2ss, c2d
import matplotlib.pyplot as plt
import numpy as np
import loopshape_tools as ls
from discreteFilter import discreteFilter
# Import the hw_16 transfer functions
import hw16_massSim as P16

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute inner and outer open-loop transfer functions
Plant = P16.P_sys
C_pid = P16.C_pid
#%% 
###########################################
# Control Design
###########################################
C = C_pid

# Because our PM starts out good enough, we will skip to adding a
# low-pass filter and lag compensator to meet the low-frequency and
# high-frequency requirements.














# ###########################################################
# # add a prefilter to eliminate the overshoot
# ###########################################################
F = ls.get_control_lpf(p=1.0)



if __name__=="__main__":
    # calculate bode plot and gain and phase margin
    # for original PID * plant dynamics
    mag, phase, omega = bode(Plant * C_pid, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{pid}P$")


    gm, pm, Wcg, Wcp = margin(Plant * C_pid)
    print("for original C_pid system:")
    if dB_flag is True:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag is False:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", gm, " Wcg: ", Wcg)


    #########################################
    #   Define Design Specifications
    #########################################
    #----------- noise specification --------
    omega_n = 1000
    mag, phase, omega = bode(Plant*C_pid, db=dB_flag, plot=False, omega=[omega_n])
    ls.add_spec_noise(gamma_n=mag[0]*0.1, omega_n=omega_n, dB_flag=dB_flag) 

    #----------- general tracking specification --------
    omega_d = 0.07
    
    # need both of these magnitudes to calculate current gamma_d, 
    # then improve it by factor of 10
    mag_PC, phase, omega = bode(Plant*C_pid, db=dB_flag, plot=False, omega=[omega_d])
    mag_P, phase, omega = bode(Plant, db=dB_flag, plot=False, omega=[omega_d])
    ls.add_spec_input_disturbance(gamma_d=mag_P/(mag_PC*10), omega_d=omega_d, system=Plant, dB_flag=dB_flag)


    #########################################
    #   Plotting routine
    #########################################

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{final}(s)P(s)$",
                             color='orange')

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    if dB_flag is True:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag is False:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    #plt.show()


    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    CLOSED_R_to_Y = (Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F * Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - no prefilter
    CLOSED_R_to_U = (C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - with prefilter
    CLOSED_R_to_U_with_F = (F*C / (1.0 + Plant * C))

    plt.figure(4)
    plt.clf()
    plt.grid(True)
    plt.subplot(311)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0, 0, 1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    else:
        plt.loglog(omega, mag, color=[0, 0, 1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F,
                             dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0, 1, 0],
            label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    else:
        plt.loglog(omega, mag, color=[0, 1, 0],
            label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    plt.ylabel('Closed-Loop Bode Plot')
    plt.grid(True)
    plt.legend()

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, color=[0,0,1],
             label='response without prefilter')
    plt.plot(T, yout_F, color=[0,1,0],
             label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')


    plt.subplot(313)
    plt.grid(True)
    _, Uout = step_response(CLOSED_R_to_U, T)
    _, Uout_F = step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout, color=[0, 0, 1],
             label='control effort without prefilter')
    plt.plot(T, Uout_F, color=[0, 1, 0],
             label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()
