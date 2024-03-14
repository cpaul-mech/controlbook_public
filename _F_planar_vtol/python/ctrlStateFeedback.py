import numpy as np
import control as cnt
import VTOLParam as P

class ctrlStateFeedback:
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
        

        wn_h = 2.2 / tr_h  # natural frequency for vertical
        wn_z = 2.2 / tr_z  # natural frequency for horizontal
        wn_th = 2.2 / tr_th
        
        des_char_poly = np.convolve(
            np.convolve([1, 2 * zeta_z * wn_z, wn_z**2], [1, 2 * zeta_th * wn_th, wn_th**2]),
            [1, 2 * zeta_h * wn_h, wn_h ** 2.0])
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 4:
            print("The system is not controllable")
        else:
            self.K = cnt.place(A, B, des_poles)
            Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
            self.kr = -1.0 / (Cr @ np.linalg.inv(A-B @ self.K) @ B)
        # print gains to terminal
        print('K: ', self.K)
        print('kr: ', self.kr)

    def update(self, z_r, x): #x = [[z], [h], [theta], [zdot], [hdot], [thetadot]]
        # Compute the state feedback controller
        F_unsat = -self.K @ x + self.kr * z_r
        F = saturate(F_unsat[0][0], P.max_thrust)
        return F


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

