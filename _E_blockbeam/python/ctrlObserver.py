import numpy as np
import control as cnt
import blockbeamParam as P

class ctrlObserver:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        tr_z = 0.85       # rise time for position
        tr_theta = 0.25    # rise time for angle
        zeta_z = 0.95  # damping ratio position
        zeta_th = 0.95  # damping ratio angle
        integrator_pole = -10.0
        tr_z_obs = tr_z/5.0 # rise time for position
        tr_theta_obs = tr_theta / 5.0 # rise time for angle
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        self.A = np.array([[0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [0.0, -P.g, 0.0, 0.0],
                      [-P.m1*P.g/(P.m2*P.ell**2/3.)+P.m1*P.ze**2, 0.0,0.,0.]]) # i suspect that subsituting z_e for zero might work?
        self.B = np.array([[0.0],
                      [0.0],
                      [0.0],
                      [P.ell/(P.m2*P.ell**2/3.)+P.m1*P.ze**2]])
        self.C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        # form augmented system
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]]) # only select z as the controlled output
        A1 = np.vstack((np.hstack((self.A, np.zeros((np.size(self.A,1),1)))), 
                        np.hstack((-Cr, np.array([[0.0]]))) ))
        B1 = np.vstack( (self.B, 0.0) )
        # gain calculation
        wn_th = 2.2 / tr_theta  # natural frequency for angle
        wn_z = 2.2 / tr_z  # natural frequency for position
        des_char_poly = np.convolve(np.convolve(
            [1, 2 * zeta_z * wn_z, wn_z**2],
            [1, 2 * zeta_th * wn_th, wn_th**2]),
            [1, -integrator_pole])
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != A1.shape[0]:
            print("The system is not controllable")
        else:
            K1 = cnt.place(A1, B1, des_poles)
            self.K = K1[0][0:4]
            self.ki = K1[0][4]
        # Calculate observer gains
        wn_z_obs = 2.2 / tr_z_obs
        wn_th_obs = 2.2 / tr_theta_obs
        des_obs_char_poly = np.convolve(
            [1, 2 * zeta_z * wn_z_obs, wn_z_obs**2],
            [1, 2 * zeta_th * wn_th_obs, wn_th_obs**2])
        des_obs_poles = np.roots(des_obs_char_poly)
        # compute the observer gains if the system is observable
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
                             [0.0], # initial estimate for theta
                             [0.0], # initial estimate for zdot
                             [0.0]]) # initial estimate for thetadot
        self.F_d1 = 0.0

    def update(self, z_r, y): #x = [[z], [theta], [zdot], [thetadot]] 
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
                     + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

