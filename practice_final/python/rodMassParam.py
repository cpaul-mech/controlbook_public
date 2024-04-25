# %%
import numpy as np
import control as cnt
# from sympy.physics.vector.printing import vpprint, vlatex
# from IPython.display import Math, display

# physical parameters
m = 0.1
ell = 0.25
b = 0.1
g = 9.8
k1 = 0.02
k2 = 0.01
tau_max = 3

# simulation parameters
t_start = 0.0
t_end = 20.0
Ts = 0.01
t_plot = 0.1

# equilibrium force
theta_e = 0.0
tau_eq = ell*g*m*np.cos(theta_e) + 0.5*k1*theta_e + k2*theta_e**3

kp = 3.0/(20.*(np.pi/180.))
ki1 = 1.0
kd = 0.228096125170475

sigma = 0.005
# define the roots of the desired closed loop system
integrator_pole = 10.0
w_n = np.sqrt(160.0*(0.02 + kp))
zeta = 0.707
des_char_poly = np.convolve([1, 2.0*zeta*w_n, w_n**2],[1, integrator_pole])
des_poles = np.roots(des_char_poly)
des_observer_poles = des_poles*5.0
A = np.array([[0.0, 1.0],[-k1/(ell**2*m), -b/(ell**2*m)]])
# print('A: ', A)
B = np.array([[0.0],[1/(ell**2*m)]])
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
# get the poles of the controller
# from sympy import symbols
# s = symbols('s')
# eig_ABK = np.linalg.det(s*np.eye(2)-A+B@K)
# print('Poles of the controller: ', eig_ABK[0])
# # get the poles of the observer
# eig_ACL = np.linalg.eig(A-L@C)
# print('Poles of the observer: ', eig_ACL[0])

# # %%
