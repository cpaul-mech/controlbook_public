import numpy as np
import VTOLParam as P
import control as cnt

class ctrlStateFeedbackIntegrator:
    def __init__(self):
        # tuning parameters
        tr_h = 2.0
        wn_h = 2.2 / tr_h
        zeta_h = 0.707
        tr_z = 2.0
        wn_z = 2.2 / tr_z
        zeta_z = 0.707
        M = 10.0
        tr_th = tr_z / M
        wn_th = 2.2 / tr_th
        zeta_th = 0.707
        z_integrator_pole = -1.0
        h_integrator_pole = -1.0
        # th_integrator_pole = -1.0
        # It is important to note that we formulate the dynamics as longitudinal dynamics
        # and lateral dynamics, but this is not strictly necessary. It may help us to see
        # and understand the effect of the two parts that are mostly decoupled (for the
        # linearized system at least). However, it is totally acceptable to also form
        # one large set of state space equations that includes A_lon and A_lat in one
        # A matrix (similary for B, C, etc.).
        # State Space Equations
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force 
        A_lon = np.array([[0.0, 1.0],
                        [0.0, 0.0]])
        B_lon = np.array([[0.0],
                        [1.0 / (P.mc + 2.0 * P.mr)]])
        C_lon = np.array([[1.0, 0.0]])
        Cr_lon = C_lon
        A1_lon = np.vstack((np.hstack((A_lon, np.zeros((np.size(A_lon,1),1)))), 
                        np.hstack((-Cr_lon, np.array([[0.0]]))) ))
        B1_lon = np.vstack( (B_lon, 0.0) )
        A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                        [0.0, -self.Fe / (P.mc + 2.0 * P.mr), -(P.mu / (P.mc + 2.0 * P.mr)), 0.0],
                        [0.0, 0.0, 0.0, 0.0]])
        B_lat = np.array([[0.0],
                        [0.0],
                        [0.0],
                        [1.0 / (P.Jc + 2 * P.mr * P.d ** 2)]])
        C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0]])
        Cr_lat = np.array([[1.0, 0.0, 0.0, 0.0]])
        A1_lat = np.vstack((np.hstack((A_lat, np.zeros((np.size(A_lat,1),1)))), 
                        np.hstack((-Cr_lat, np.array([[0.0]]))) ))
        B1_lat = np.vstack( (B_lat, 0.0) )
        # gain calculation
        des_char_poly_lon = np.convolve([1.0, 2.0 * zeta_h * wn_h, wn_h ** 2], [1.0, -h_integrator_pole])
        des_poles_lon = np.roots(des_char_poly_lon)
        des_char_poly_lat = np.convolve(np.convolve([1.0, 2.0 * zeta_z * wn_z, wn_z ** 2],
                                        [1.0, 2.0 * zeta_th * wn_th, wn_th ** 2]), [1.0, -z_integrator_pole])
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
        print('K_lon: ', self.K_lon)
        print('kr_lon: ', self.ki_lon)
        print('K_lat: ', self.K_lat)
        print('kr_lat: ', self.ki_lat)
        # variables to implement integrator
        self.integrator_z = 0.0
        self.error_z_d1 = 0.0
        self.integrator_h = 0.0
        self.error_h_d1 = 0.0
        

    def update(self, r, x): #x = [[z], [h], [theta], [zdot], [hdot], [thetadot]], r = [[z_r], [h_r]]
        z_r = r[0][0]
        h_r = r[1][0]
        z = x[0][0]
        h = x[1][0]
        theta = x[2][0]
        # integrate error
        error_z = z_r - z
        error_h = h_r - h
        self.integrator_z = self.integrator_z \
            + (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        self.integrator_h = self.integrator_h \
            + (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h
        # Construct the states
        x_lon = np.array([[x[1,0]], [x[3,0]]])
        x_lat = np.array([[x[0,0]], [x[2,0]], [x[3,0]], [x[5,0]]])
        # Compute the state feedback controllers
        F_tilde = -self.K_lon @ x_lon - self.ki_lon * self.integrator_h
        F = self.Fe / np.cos(theta) + F_tilde[0]
        tau = -self.K_lat @ x_lat - self.ki_lat * self.integrator_z
        return np.array([[F], [tau[0]]])


def saturate(u, limit):
    if abs(u) > limit:
        u = limit*np.sign(u)
    return u
