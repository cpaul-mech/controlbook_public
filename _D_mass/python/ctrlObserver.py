import numpy as np
import control as cnt
import massParam as P

class ctrlObserver:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        wn_z = 2.0      # rise time for position
        zeta_z = 0.707  # damping ratio position
        integrator_pole = -5.0
        wn_z_obs = 10.0*wn_z # no idea if this is a good value
        zeta_z_obs = 0.707
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        self.A = np.array([[0.0, 1.0],[-P.k/P.m, -P.b/P.m]])
        self.B = np.array([[0.0],[1/P.m]])
        self.C = np.array(([[1.0, 0.0]]))
        # form augamented system:
        Cr = np.array([[1.0, 0.0]]) # I don't think that this C needs to change.
        A1 = np.vstack((np.hstack((self.A, np.zeros((np.size(self.A,1),1)))),  # A1 doesn't change.
                        np.hstack((-Cr, np.array([[0.0]]))) ))
        B1 = np.vstack( (self.B, 0.0) )
        # gain calculation
        des_char_poly = np.convolve([1, 2 * zeta_z * wn_z, wn_z**2],[1, -integrator_pole])
        des_poles = np.roots(des_char_poly)
        # Compute the control gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != A1.shape[0]:
            print("The system is not controllable")
        else:
            K1 = cnt.place(A1, B1, des_poles)
            self.K = K1[0,0:2]
            self.ki = K1[0,2]
        # compute observer gains
        des_obs_char_poly = [1, 2*zeta_z_obs*wn_z_obs, wn_z_obs**2]
        des_obs_poles = np.roots(des_obs_char_poly) # I don't think that we need to use convolve bc few poles.
        if np.linalg.matrix_rank(cnt.ctrb(self.A.T, self.C.T)) != self.A.shape[0]:
            print("The system is not observable")
        else:
            self.L = cnt.place(self.A.T, self.C.T, des_obs_poles).T
        # print gains to terminal
        print('K: ', self.K)
        print('ki: ', self.ki)
        print('L^T: ', self.L.T)
        # variables to implement integrator
        self.integrator_z = 0.0 # integrator
        self.error_z_d1 = 0.0 # error signal delayed by 1 sample
        # estimated state variables
        self.x_hat = np.array([[0.0], # initial estimate for z
                                 [0.0]]) # initial estimate for zdot
        self.F_d1 = 0.0 # Computed Force, delayed by 1 sample

    def update(self, z_r, y): # state is [[z]]
        # update the observer and extract z_hat
        x_hat = self.update_observer(y)
        z_hat = x_hat[0][0]
        # integrate error
        error_z = z_r - z_hat
        self.integrator_z = self.integrator_z \
            + (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        # Compute the state feedback controller
        F_unsat = -self.K @ x_hat - self.ki * self.integrator_z
        F = saturate(F_unsat[0], P.F_max)
        self.F_d1 = F
        return F, x_hat
    
    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + P.Ts/2*F1, y_m)
        F3 = self.observer_f(self.x_hat + P.Ts/2*F2, y_m)
        F4 = self.observer_f(self.x_hat + P.Ts*F3, y_m)
        self.x_hat += P.Ts/6*(F1 + 2*F2 + 2*F3 + F4)
        return self.x_hat
    
    def observer_f(self, x_hat, y_m):
        # xhat_dot = A*xhat + B*u + L(y_m - C*xhat)
        xhat_dot = self.A @ x_hat \
                     + self.B * self.F_d1 \
                     + self.L * (y_m - self.C @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
