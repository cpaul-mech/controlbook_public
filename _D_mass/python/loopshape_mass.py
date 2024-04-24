import hw16 as P16
import matplotlib.pyplot as plt
from control import tf, bode, margin, mag2db, tf2ss, step_response
import numpy as np
import loopshape_tools as lt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# assigning plant and controller from past HW
# (to make sure we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid

#######################################################################
#   Control Design
#######################################################################
C = C_pid

# integral control: increase steady state tracking and dist rejection
ki = 5.0  # frequency at which integral action ends
C_int = lt.get_control_integral(ki)

# phase lead (|p|>|z|): increase PM (stability)
# low frequency gain = K*z/p
# high frequency gain = K
w_lead = 3.0  # location of maximum frequency bump, approx halfway between other constraints
phi_max = 80.0 * np.pi / 180.0  # specify phase margin we want to add
M = (1 + np.sin(phi_max)) / (1 - np.sin(phi_max))  # calculate necessary lead ratio (M)
C_lead = lt.get_control_lead(w_lead, M)

# find gain to set crossover frequency w_lead = w_co = 3 rad/s
mag, phase, omega = bode(Plant * C * C_int * C_lead, dB=dB_flag, omega=[w_lead], plot=False)
C_k = lt.get_control_proportional(1 / mag[0])

# we are close on the high freq noise requirement, so let's add a lpf just in case
p = 10.0
C_lpf = lt.get_control_lpf(p)

# we are also close for the low-frequency tracking requirement, but let's add a lag filter
C_lag = lt.get_control_lag(0.1, 10)

# calculate the final controller form
C = C*C_int*C_lead*C_k*C_lpf*C_lag

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
F = 1.0
# low pass filter
p = 2.0 
LPF = lt.get_control_lpf(p) 
F = F * LPF

##############################################
#  Convert Controller to State Space Equations if following method in 18.1.7
##############################################
C_ss = tf2ss(C)  # convert to state space
F_ss = tf2ss(F)  # convert to state space



if __name__ == "__main__":
    # calculate bode plot and gain and phase margin for original PID * plant dynamics
    mag, phase, omega = bode(Plant * C_pid, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{pid}(s)P(s)$")

    gm, pm, Wcg, Wcp = margin(Plant * C_pid)
    print("for original C_pid system:")
    if dB_flag == True:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag == False:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #########################################
    #   Define Design Specifications
    #########################################

    # ----------- noise specification --------
    omega_n = 500  # attenuate noise above this frequency
    gamma_n = 0.001  # attenuate noise by this amount
    lt.add_spec_noise(gamma_n, omega_n, dB_flag)

    # ----------- general tracking specification --------
    omega_r = 0.1  # track signals below this frequency
    gamma_r = 0.03  # tracking error below this value
    lt.add_spec_ref_tracking(gamma_r, omega_r, dB_flag)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True, label="$C_{final}(s)P(s)$")

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    if dB_flag == True:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag == False:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    plt.show()

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
    CLOSED_R_to_U_with_F = (F * C / (1.0 + Plant * C))

    fig = plt.figure()
    plt.grid(True)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=True,
                             color=[0, 0, 1], label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F, dB=dB_flag, plot=True,
                             color=[0, 1, 0], label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    fig.axes[0].set_title('Closed-Loop Bode Plot')
    fig.axes[0].legend()

    plt.figure()
    plt.subplot(211), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, 'b', label='response without prefilter')
    plt.plot(T, yout_F, 'g', label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')

    plt.subplot(212), plt.grid(True)
    _, Uout_no_F = step_response(CLOSED_R_to_U, T)
    _, Uout_F = step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout_no_F, color='b', label='control effort without prefilter')
    plt.plot(T, Uout_F, color='g', label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()
