import sys
import serial

import arduinoCommunication as ardCom
import hummingbirdParam as P
import tkinter as tk
import numpy as np

from hummingbirdAnimation import HummingbirdAnimation
from threading import Thread
from HB_GUI_V2 import HB_GUI


# ----------------Global Variables--------------------
animate = False
t = P.t_start

# ----------------Check for Serial Connection--------------------
try:
    ser = serial.Serial('COM3', 9600)
except:
    print()
    print("Hummingbird not connected")
    print()
    sys.exit()   

# ----------------Start Arduino Thread--------------------
thread = Thread(target=lambda: ardCom.read_from_arduino(ser))
thread.daemon = True  # This thread dies when the main thread (program) exits
thread.start()

# ----------------Initialize the Root Event Handler--------------------
root = tk.Tk()
root.title("Controller Interface")
GUILayout = HB_GUI(root,
                   km_slider = True,
                   encoder_val = True,
                   lonPID = True,
                   )
    
# ----------------Initialize Animation--------------------
if animate:
    animation = HummingbirdAnimation()
    def update_animation():
        global t
        t = t + P.Ts_Animation
        t = np.mod(t,29)
        animation.update(t,np.array([[P.phi],[P.theta],[P.psi]]))
        root.after(int(P.Ts_Animation*1000), update_animation)
    update_animation()

# ----------------GUI Communication--------------------
def update_GUI():
    GUILayout.GUI_Values(P.phi,P.theta,P.psi)
    ardCom.send_gains(ser)
    root.after(int(P.Ts_GUI*1000),update_GUI)    
update_GUI()

def flip_theta():
    GUILayout.Flip_Theta()
    root.after(10000,flip_theta)
flip_theta()

# ----------------Start--------------------
root.mainloop()

# ----------------Close--------------------
ser.close()
