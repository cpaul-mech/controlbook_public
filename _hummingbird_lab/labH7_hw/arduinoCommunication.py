import hummingbirdParam as P
import numpy as np
import time

def read_from_arduino(ser):
    while True:
        if ser.inWaiting() > 0:
            x = ser.read().decode()
            if x == chr(P.startMarker):
                P.reading = True
                P.bytesRec = 0
                P.tempBuffer = ""

            if P.reading:
                P.tempBuffer = P.tempBuffer + x
                P.bytesRec += 1

            if x == chr(P.endMarker):
                P.reading = False
                stringRec = ""
                for i in range(1,P.bytesRec - 1):
                    stringRec += P.tempBuffer[i]

                f_pos = stringRec.find('F')
                t_pos = stringRec.find('T')
                s_pos = stringRec.find('S')

                if f_pos != -1 and t_pos != -1 and s_pos != -1:
                    P.phi = float(stringRec[f_pos + 1:t_pos])
                    P.theta = float(stringRec[t_pos + 1:s_pos])
                    P.psi = float(stringRec[s_pos + 1:])

def send_gains(ser):
    pwm_send = chr(P.startMarker) + "R" + str(P.theta_ref) + "K" + str(P.km) + "P" + str(P.theta_kp) + "I" + str(P.theta_ki) + "D" + str(P.theta_kd) + chr(P.endMarker)
    byte_pwm = pwm_send.encode()
    ser.write(byte_pwm)
    P.communicated = True
        