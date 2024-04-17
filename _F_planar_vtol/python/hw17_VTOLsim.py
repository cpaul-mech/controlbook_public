# Satellite Parameter File
import VTOLParam as P
import hw16_VTOLsim as P16
from control import *
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Import transfer functions from previous HW solution
P_lon = P16.P_lon
C_lon = P16.C_lon
P_lat_in = P16.P_lat_in
C_lat_in = P16.C_lat_in
P_lat_out = P16.P_lat_out
C_lat_out = P16.C_lat_out


if __name__ == '__main__':
    
    # we have to define the frequencies, or we get non-smooth
    # bode plots.
    omegas = np.logspace(-2, 3, 1000)

    ##################################################
    ########### Longitudinual ###########################
    # Calculate the phase and gain margins
    if dB_flag:
        gm, pm, Wcg, Wcp = margin(P_lon * C_lon)
        gm = mag2db(gm)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    else:
        gm, pm, Wcg, Wcp = margin(P_lon * C_lon)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    # display bode plots of transfer functions
    fig1 = plt.figure()

    # this makes two bode plots for open and closed loop
    bode(P_lon * C_lon, omega=omegas, dB=dB_flag,
         label='$CP$ - Open-loop')
    bode(P_lon * C_lon / (1 + P_lon * C_lon),
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
    fig1.axes[0].set_title(r'VTOL longitudinal C(s)*P(s)  ' +
                           'GM:' + str(round(gm, 2)) +
                           ', PM:' + str(round(pm, 2)))

    ##################################################
    ########### Inner loop of lateral ###########################
    # Calculate the phase and gain margins
    if dB_flag:
        gm, pm, Wcg, Wcp = margin(P_lat_in *C_lat_in)
        gm = mag2db(gm)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    else:
        gm, pm, Wcg, Wcp = margin(P_lat_in *C_lat_in)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    # display bode plots of transfer functions
    fig2 = plt.figure()

    # this makes two bode plots for open and closed loop
    bode(P_lat_in *C_lat_in, omega=omegas, dB=dB_flag,
         label='$C_{in}P_{in}$ - Open-loop')
    bode(P_lat_in *C_lat_in / (1 + P_lat_in *C_lat_in),
         omega=omegas, dB=dB_flag,
         label=r'$\frac{P_lat_{in}C_lat_{in}}{1+P_lat_in{in}C_lat_{in}}$'
               + '- Closed-loop')

    # now we can add lines to show where we calculated the GM and PM
    gm_line = fig2.axes[0].plot([Wcg, Wcg],
                                plt.ylim(), 'k--', label='GM')
    gm_line[0].set_label('GM')
    fig2.axes[0].legend()
    fig2.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
    plt.legend()

    # setting axis title
    fig2.axes[0].set_title(r'VTOL Lateral Inner Loop ($\theta$) - ' +
                           'GM:' + str(round(gm, 2)) +
                           ', PM:' + str(round(pm, 2)))
    
    ##################################################
    ########### VTOL Lateral Outer loop ###########################
    # Calculate the phase and gain margins
    if dB_flag:
        gm, pm, Wcg, Wcp = margin(P_lat_out * C_lat_out)
        gm = mag2db(gm)
        print("Outer Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    else:
        gm, pm, Wcg, Wcp = margin(P_lat_out * C_lat_out)
        print("Outer Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    # display bode plots of transfer functions
    fig3 = plt.figure()

    # this makes two bode plots for open and closed loop
    bode(P_lat_out * C_lat_out, omega=omegas, dB=dB_flag,
         label='$C_lat_out P_lat_out$ - Open-loop')
    bode(P_lat_out * C_lat_out / (1 + P_lat_out * C_lat_out),
         omega=omegas, dB=dB_flag,
         label=r'$\frac{P_lat_{out}C_lat_{out}}{1+P_lat_{out}C_lat_{out}}$'
               + ' - Closed-loop')

    # now we can add lines to show where we calculated the GM and PM
    gm_line = fig3.axes[0].plot([Wcg, Wcg],
                                plt.ylim(), 'k--', label='GM')
    gm_line[0].set_label('GM')
    fig3.axes[0].legend()
    fig3.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
    plt.legend()

    # setting axis title
    fig3.axes[0].set_title(r'VTOL Lateral Outer Loop - ' +
                           'GM:' + str(round(gm, 2)) +
                           ', PM:' + str(round(pm, 2)))


    print('Close window to end program')
    plt.show()
