import numpy as np
import VTOLParam as P
import control as cnt

class ctrlObserver:
    def __init__(self):
        # initialize wind force (this term was always in the dynamics solution, but may not be
        # included in your own, so please check).
        P.F_wind = 0.1
        
        # tuning parameters
        wn_h = 1.0
        zeta_h = 0.95
        wn_z = 0.9905
        zeta_z = 0.95
        wn_th = 13.3803
        zeta_th = 0.95
        z_integrator_pole = 1.0
        h_integrator_pole = 1.0
        # define observer paramters
        wn_h_obs = 5.0*wn_h
        zeta_h_obs = 0.707
        wn_z_obs = 5.0*wn_z
        zeta_z_obs = 0.707
        wn_th_obs = 5.0*wn_th
        zeta_th_obs = 0.707

        # State Space Equations
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force 
        # x = [[h], [hdot]]
        self.A_lon = np.array([[0.0, 1.0],
                        [0.0, 0.0]])
        self.B_lon = np.array([[0.0],
                        [1.0 / (P.mc + 2.0 * P.mr)]])
        self.C_lon = np.array([[1.0, 0.0]])
        # Cr_lon = C_lon
        A1_lon = np.vstack((
                np.hstack((self.A_lon, np.zeros((2,1)))),
                np.hstack((-self.C_lon, np.zeros((1,1))))))
        B1_lon = np.vstack((self.B_lon, np.zeros((1,1))))
        # x = [[z], [theta], [zdot], [thetadot]]
        self.A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                        [0.0, -self.Fe / (P.mc + 2.0 * P.mr), -(P.mu / (P.mc + 2.0 * P.mr)), 0.0],
                        [0.0, 0.0, 0.0, 0.0]])
        self.B_lat = np.array([[0.0],
                        [0.0],
                        [0.0],
                        [1.0 / (P.Jc + 2 * P.mr * P.d ** 2)]])
        self.C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0]])
        # Cr_lat = np.array([[1.0, 0.0, 0.0, 0.0]])
    
        A1_lat = np.vstack((
                np.hstack((self.A_lat, np.zeros((4,1)))),
                np.hstack((-self.C_lat[0:1], np.zeros((1,1))))))
        B1_lat = np.vstack((self.B_lat, np.zeros((1,1))))
        # gain calculation
        des_char_poly_lon = np.convolve([1.0, 2.0 * zeta_h * wn_h, wn_h ** 2],
                                        [1.0, h_integrator_pole])
        des_poles_lon = np.roots(des_char_poly_lon)
        des_char_poly_lat = np.convolve(
            np.convolve([1.0, 2.0 * zeta_z * wn_z, wn_z ** 2],
                        [1.0, 2.0 * zeta_th * wn_th, wn_th ** 2]), 
                        [1.0, z_integrator_pole])
        des_poles_lat = np.roots(des_char_poly_lat)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
            print("The longitudinal system is not controllable")
        else:
            K1_lon = cnt.place(A1_lon, B1_lon, des_poles_lon)
            self.K_lon = K1_lon[0][0:2]
            self.ki_lon = K1_lon[0][2]
        if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
            print("The lateral system is not controllable")
        else:
            K1_lat = cnt.place(A1_lat, B1_lat, des_poles_lat)
            self.K_lat = K1_lat[0][0:4]
            self.ki_lat = K1_lat[0][4]
        # Compute Observer Gains
        des_obs_char_poly_lon = [1.0, 2.0 * zeta_h_obs * wn_h_obs, wn_h_obs ** 2]
        des_obs_poles_lon = np.roots(des_obs_char_poly_lon)
        des_obs_char_poly_lat = np.convolve([1.0, 2.0 * zeta_z_obs * wn_z_obs, wn_z_obs ** 2],
                                            [1.0, 2.0 * zeta_th_obs * wn_th_obs, wn_th_obs ** 2])
        des_obs_poles_lat = np.roots(des_obs_char_poly_lat)
        if np.linalg.matrix_rank(cnt.ctrb(self.A_lon.T, self.C_lon.T)) != 2:
            print("The longitudinal observer is not controllable")
        else:
            self.L_lon = cnt.place(self.A_lon.T, self.C_lon.T, des_obs_poles_lon).T
        if np.linalg.matrix_rank(cnt.ctrb(self.A_lat.T, self.C_lat.T)) != 4:
            print("The lateral observer is not controllable")
        else:
            self.L_lat = cnt.place(self.A_lat.T, self.C_lat.T, des_obs_poles_lat).T
        # print gains to terminal
        print('K_lon: ', self.K_lon)
        print('ki_lon: ', self.ki_lon)
        print('K_lat: ', self.K_lat)
        print('ki_lat: ', self.ki_lat)
        print('L_lon^T: ', self.L_lon.T)
        print('L_lat^T: ', self.L_lat.T)
        # variables to implement integrator
        self.integrator_z = 0.0
        self.error_z_d1 = 0.0
        self.integrator_h = 0.0
        self.error_h_d1 = 0.0
        # saturation limits local to the class
        self.F_limit = P.max_thrust * 2.0
        self.tau_limit = P.max_thrust * P.d * 2.0
        # Estimated state variables
        self.x_hat_lon = np.array([[0.0], # initial estimate for h
                                   [0.0]]) # initial estimate for hdot
        self.x_hat_lat = np.array([[0.0], # initial estimate for z
                                   [0.0], # initial estimate for theta
                                   [0.0], # initial estimate for zdot
                                   [0.0]]) # initial estimate for thetadot
        self.F_d1 = 0.0
        self.tau_d1 = 0.0
        

    def update(self, r, y): #x = [[z], [h], [theta], [zdot], [hdot], [thetadot]], r = [[z_r], [h_r]]
        # y = [[z]]
        # update the observer for 
        y_lon = np.array([[y[1][0]]])
        x_hat_lon = self.update_observer_lon(y_lon)
        h_hat = x_hat_lon[0][0]
        y_lat = np.array([[y[0][0]], [y[2][0]]])
        x_hat_lat = self.update_observer_lat(y_lat)
        z_hat = x_hat_lat[0][0]
        theta_hat = x_hat_lat[1][0]
        z_r = r[0][0]
        h_r = r[1][0]
        # integrate error
        error_z = z_r - z_hat
        self.integrator_z += (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        error_h = h_r - h_hat
        self.integrator_h += (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h

        # Construct the states
        # I don't think this is necessary any more now!!

        # Compute the state feedback controllers
        F_tilde = -self.K_lon @ x_hat_lon - self.ki_lon * self.integrator_h
        F = self.Fe / np.cos(theta_hat) + F_tilde[0]
        F_sat = saturate(F, self.F_limit)
        self.integratorAntiWindup(F_sat, F, self.ki_lon, self.integrator_h)
        tau = -self.K_lat @ x_hat_lat - self.ki_lat * self.integrator_z
        tau_sat = saturate(tau[0], self.tau_limit)
        self.integratorAntiWindup(tau_sat, tau, self.ki_lat, self.integrator_z)
        self.F_d1 = F_sat
        self.tau_d1 = tau_sat
        return np.array([[F_sat], [tau_sat]]), x_hat_lon, x_hat_lat #make sure to deal with this output in the main file

    def integratorAntiWindup(self, u_sat, u_unsat, ki, integrator):
            if ki != 0.0:
                integrator = integrator + P.Ts/ki*(u_sat-u_unsat)
    
    def update_observer_lon(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lon(self.x_hat_lon, y_m)
        F2 = self.observer_f_lon(self.x_hat_lon + P.Ts/2*F1, y_m)
        F3 = self.observer_f_lon(self.x_hat_lon + P.Ts/2*F2, y_m)
        F4 = self.observer_f_lon(self.x_hat_lon + P.Ts*F3, y_m)
        self.x_hat_lon += P.Ts/6*(F1 + 2*F2 + 2*F3 + F4)
        return self.x_hat_lon
    
    def observer_f_lon(self, x_hat_lon, y_m):
        # xhat_dot = A*xhat + B*u + L(y_m - C*xhat)
        xhat_lon_dot = self.A_lon @ x_hat_lon \
                     + self.B_lon * self.F_d1 \
                     + self.L_lon @ (y_m - self.C_lon @ x_hat_lon) #check the second term for if * needs to be @
        return xhat_lon_dot
    
    def update_observer_lat(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lat(self.x_hat_lat, y_m)
        F2 = self.observer_f_lat(self.x_hat_lat + P.Ts/2*F1, y_m)
        F3 = self.observer_f_lat(self.x_hat_lat + P.Ts/2*F2, y_m)
        F4 = self.observer_f_lat(self.x_hat_lat + P.Ts*F3, y_m)
        self.x_hat_lat += P.Ts/6*(F1 + 2*F2 + 2*F3 + F4)
        return self.x_hat_lat
    
    def observer_f_lat(self, x_hat_lat, y_m):
        # xhat_dot = A*xhat + B*u + L(y_m - C*xhat)
        xhat_lat_dot = self.A_lat @ x_hat_lat \
                     + self.B_lat * self.tau_d1 \
                     + self.L_lat @ (y_m - self.C_lat @ x_hat_lat)
        return xhat_lat_dot
                
def saturate(u, limit):
    if abs(u) > limit:
        u = limit*np.sign(u)
    return u

