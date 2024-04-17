# Satellite Parameter File
import massParam as P
import hw16_massSim as P16
from control import *
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute inner and outer open-loop transfer functions
P_sys = P16.P_sys
C_pid = P16.C_pid

if __name__ == '__main__':
    
    # we have to define the frequencies, or we get non-smooth
    # bode plots.
    omegas = np.logspace(-2, 3, 1000)

    ##################################################
    ########### Inner loop ###########################
    # Calculate the phase and gain margins
    if dB_flag:
        gm, pm, Wcg, Wcp = margin(P_sys * C_pid)
        gm = mag2db(gm)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    else:
        gm, pm, Wcg, Wcp = margin(P_sys * C_pid)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    # display bode plots of transfer functions
    fig1 = plt.figure()

    # this makes two bode plots for open and closed loop
    bode(P_sys * C_pid, omega=omegas, dB=dB_flag,
         label='$CP$ - Open-loop')
    bode(P_sys * C_pid / (1 + P_sys * C_pid),
         omega=omegas, dB=dB_flag,
         label=r'$\frac{PC}{1+PC}$'
               + '- Closed-loop')

    # now we can add lines to show where we calculated the GM and PM
    gm_line = fig1.axes[0].plot([Wcg, Wcg],
                                plt.ylim(), 'k--', label='GM')
    gm_line[0].set_label('GM')
    fig1.axes[0].legend()
    fig1.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
    plt.legend()

    # setting axis title
    fig1.axes[0].set_title(r'Mass Spring C(s)*P(s)  ' +
                           'GM:' + str(round(gm, 2)) +
                           ', PM:' + str(round(pm, 2)))

    print('Close window to end program')
    plt.show()
