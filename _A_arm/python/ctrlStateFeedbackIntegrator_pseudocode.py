import numpy as np
import control as cnt
import armParam as P


class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        #  tuning parameters
        tr = 0.4
        zeta = 0.707
        integrator_pole = -5.
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([[0.0, 1.0],
                      [0.0, -1.0 * P.b / P.m / (P.ell**2)]])
        B = np.array([[0.0],
                      [3.0 / P.m / (P.ell**2)]])        
        Cr = np.array([[1.0, 0.0]])
        # form augmented system
        A1 = np.vstack((np.hstack((A, np.zeros((np.size(A,1),1)))), 
                        np.hstack((-Cr, np.array([[0.0]]))) ))
        B1 = np.vstack( (B, 0.0) )
        # gain calculation
        wn = 2.2 / tr  # natural frequency
        #wn = 0.5*np.pi/(tr*np.sqrt(1-zeta**2)) # natural frequency
        des_char_poly = np.convolve([1, 2 * zeta * wn, wn**2], 
                                    [1, -integrator_pole])
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
            print("The system is not controllable")
        else:
            K1 = cnt.place(A1, B1, des_poles)
            self.K = K1[0][0:2]
            self.ki = K1[0][2]
        print('K: ', self.K)
        print('ki ', self.ki)
        print(des_poles)
        #--------------------------------------------------
        # variables to implement integrator
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        Cm = Cr  # Might be Cr, but might be not in other cases.
        # check observability
        O = cnt.ctrb(A.T, Cm.T)
        if np.linalg.matrix_rank(O) != 2:
            print("System Not Observable")
        
        # define desired observer poles
        s1 = -2.0 + 1.0.J
        s2 = -2.0 - 1.0.J
        p1 = -1
        des_obsv_poles = ______ # N times faster than controller poles (check real-part )
        L = cnt.place(A.T, Cm.T, des_obsv_poles).T # the transpose at the end is necessary.


    def update(self, theta_r, y):
        x_hat = self.update_observer(y) #update ovserver will update the observer using the rk4step.

        theta = x_hat[0][0]
        # integrate error
        error = theta_r - theta
        self.integrator = self.integrator \
                          + (P.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error
        # compute feedback linearizing torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat- self.ki * self.integrator
        # compute total torque
        tau = saturate(tau_fl + tau_tilde[0], P.tau_max)
        return tau, x_hat


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

