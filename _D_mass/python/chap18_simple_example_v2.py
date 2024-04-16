#####################################################################
####################### Chapter 18 ##################################
#####################################################################
from control import bode, tf, margin
import matplotlib.pyplot as plt
import numpy as np
import loopshape_tools as lt
from discreteFilter import discreteFilter

##################
# define plant   #
##################
Plant = tf([1],[1,1]) #transfer function for plant

##################
# control design #
##################
# this is an empty controller "C" that doesn't do anything yet. We will add
# more terms below, one at a time. 
C = tf([1.0], [1.0])

#integral control
k_I = 0.4
Integrator = lt.get_control_integral(k_I)
C = C*Integrator

#proportional control
kp = 1/0.77  # 1/(current Mag. ratio at desired crossover freq.)
C = C*kp

#phase lag
z = 0.8
M = 100.0
# Lag = tf([1, z], [1, z/M])
Lag = lt.get_control_lag(z, M)
C = C*Lag

#low-pass filter
p = 5
#LPF = tf([p], [1, p])
LPF = lt.get_control_lpf(p)
C = C*LPF


# this shows how to now turn our continuous-time controller transfer function
# into a discrete-time controller/filter. This is for a sampling time of 0.01 sec. 
controller = discreteFilter(C.num, C.den, 0.01)

# to use this function, it needs to be included or defined in a controller and will 
# return the desired output when called as follows:
#   u = controller.update(error)

if __name__=="__main__":

    dB_flag = False

    # calculate bode plot and gain and phase margin
    # plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True, label='Plant - P(s)')

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #########################################
    #   Define Design Specifications
    #########################################
    #----------- ramp input error -----------
    gamma_r = 0.03
    lt.add_spec_tracking_ramp(gamma_r, dB_flag)

    #----------- noise specification --------
    omega_n = 10    # attenuate noise above this frequency
    gamma_n = 0.1  # attenuate noise by this amount
    lt.add_spec_noise(gamma_n, omega_n, dB_flag)

    #----------- disturbance rejection -------
    gamma_d_in = 0.1 # attenuate disturbance input by this amount
    omega_d_in = 0.1 # attenuate disturbance below this frequency
    lt.add_spec_input_disturbance(gamma_d_in, omega_d_in, Plant, dB_flag)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant*C, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             label='P(s)C(s)',
                             plot=True,
                             margins=True)
 
    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    fig.axes[0].grid(True)
    fig.axes[1].grid(True)
    fig.axes[0].set_title('Bode Diagram')
    plt.show()
