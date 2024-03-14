import numpy as np
import control as cnt
import massParam as P

class ctrlStateFeedback:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        tr_z = 0.8        # rise time for position
        zeta_z = 0.707  # damping ratio position
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([[0.0, 1.0],[-P.k/P.m, -P.b/P.m]])
        B = np.array([[0.0],[1/P.m]])
        C = np.array(([1.0, 0.0]))
        # gain calculation
        wn_z = 2.2 / tr_z  # natural frequency for position
        des_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        des_poles = np.roots(des_char_poly)
        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 2:
            print("The system is not controllable")
        else:
            self.K = cnt.place(A, B, des_poles)
            Cr = np.array([[1.0, 0.0]])
            self.kr = -1.0 / (Cr @ np.linalg.inv(A-B @ self.K) @ B)
        # print gains to terminal
        print('K: ', self.K)
        print('kr: ', self.kr)

    def update(self, z_r, x): # state is [[z]]
        # Compute the state feedback controller
        F_unsat = -self.K @ x + self.kr * z_r
        F = saturate(F_unsat[0][0], P.F_max)
        return F


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

