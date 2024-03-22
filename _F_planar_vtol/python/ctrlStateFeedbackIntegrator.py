import numpy as np
import control as cnt
import VTOLParam as P

class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        # tuning parameters
        tr_h = 3.0  # rise time for altitude - original (3.0)
        zeta_h = 0.95  # damping ratio for altitude - original (0.707)
        tr_z = 3.0  # rise time for outer lateral loop (position) - original
        M = 10.0  # time separation between inner and outer lateral loops
        zeta_z = 0.9  # damping ratio for outer lateral loop
        zeta_th = 0.9  # damping ratio for inner lateral loop
        tr_th = tr_z / M
        zi_pole = -5.0
        hi_pole = -5.0
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                      [0.0, 0.0, -P.g, -P.mu/(P.mc+2.0*P.mr), 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        B = np.array([[0.0, 0.0],
                      [0.0, 0.0],
                      [0.0, 0.0],
                      [0.0, 0.0],
                      [1.0/(P.mc+2.0*P.mr), 0.0],
                      [0.0, 1.0/(P.Jc+2.0*P.mr*P.d**2)]])
        C = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])
        Cr = C
        A1 = np.vstack((np.hstack((A, np.zeros((np.size(A,1),1)))), 
                        np.hstack((-Cr, np.array([[0.0]]))) ))
        B1 = np.vstack( (B, 0.0) )

        wn_h = 2.2 / tr_h  # natural frequency for vertical
        wn_z = 2.2 / tr_z  # natural frequency for horizontal
        wn_th = 2.2 / tr_th
        
        des_char_poly = np.convolve(
                            np.convolve(
                                np.convolve([1, 2 * zeta_z * wn_z, wn_z**2], [1, 2 * zeta_th * wn_th, wn_th**2]),
                                        [1, 2 * zeta_h * wn_h, wn_h ** 2.0]), 
                        [1, -zi_pole, -hi_pole])
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 7: #maybe change this to 7 if necessary
            print("The system is not controllable")
        else:
            K1 = cnt.place(A1, B1, des_poles)
            self.K = K1[0][0:6]
            self.ki = K1[0][6]
        # print gains to terminal
        print('K: ', self.K)
        print('ki: ', self.ki)
        # variables to implement integrator
        self.integrator_z = 0.0
        self.error_z_d1 = 0.0
        self.integrator_h = 0.0
        self.error_h_d1 = 0.0

    def update(self, z_r, h_r, x): #x = [[z], [h], [theta], [zdot], [hdot], [thetadot]]
        z = x[0][0]
        h = x[1][0]
        # integrate error
        error_z = z_r - z
        error_h = h_r - h
        self.integrator_z = self.integrator_z \
            + (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z
        self.integrator_h = self.integrator_h \
            + (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h
        # compute the state feedback controller
        F_unsat = -self.K @ x - self.ki * np.array([[self.integrator_z], [self.integrator_h]])
        F = saturate(F_unsat[0][0], P.max_thrust)
        return F
        # Compute the state feedback controller


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

