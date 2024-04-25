#%%
import numpy as np
import control as cnt

# one of these two forms for importing control library functions may be helpful.
#import control as cnt
#from control import tf, bode, etc.

# physical parameters
g = 9.8
theta = 45.0*np.pi/180.0   #this variable just defines the slope of the block, and may not be used explicitly
m = 0.5
k1 = 0.05
k2 = 0.02
F_max = 5.0
b = 0.1

# simulation parameters
t_start = 0.0
t_end = 20.0
Ts = 0.01
t_plot = 0.1
sigma = 0.05
z_e = 0.0

F_eq = -g*m/np.sqrt(2) + k1*z_e + k2*z_e**3

b0 = 1.0/m
a1 = b/m
a0 = k1 + k2
k_p = 5.0
w_n = np.sqrt(a0+b0*k_p)
zeta = 0.707
k_d = (2.0*zeta*w_n - a1)/b0
print("k_d = ", k_d)

k_i1 = 0.0
# set poles to s = -2+/-2j
integrator_pole = -5.0
des_poles = np.array([integrator_pole, -2.0+2.J, -2.0-2.J])
des_observer_poles = des_poles*5.0
A = np.array([[0.0, 1.0],[(-k1-3.0*k2*z_e**2)/(m), -b/m]])
B = np.array([[0.0],[1/m]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])
A1 = np.vstack((np.hstack((A, np.zeros((2,1)))), 
                        np.hstack((-C, np.zeros((1,1)))) ))
B1 = np.vstack((B, 0.0))
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print("The system is not controllable")
else:
    K1 = cnt.place(A1, B1, des_poles)
    K = K1[0][0:2]
    ki2 = K1[0][2]
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 2:
    print("The system is not observerable")
else:
    L = cnt.acker(A.T, C.T, des_observer_poles).T
print('K: ', K)
print('ki ', ki2)
print('L^T: ', L.T)
# %%
