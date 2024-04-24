import massParam as P
import loopshape_mass as L
import numpy as np
from digitalFilter import digitalFilter

class ctrlLoopshape :
    def __init__(self, method="digital_filter"):
        if method == "state_space":
            self.x_C = np.zeros((L.C_ss.A.shape[0], 1))
            self.x_F = np.zeros((L.F_ss.A.shape[0], 1))
            self.A_C = L.C_ss.A
            self.B_C = L.C_ss.B
            self.C_C = L.C_ss.C
            self.D_C = L.C_ss.D
            self.A_F = L.F_ss.A
            self.B_F = L.F_ss.B
            self.C_F = L.F_ss.C
            self.D_F = L.F_ss.D
            self.N = 10  #number of Euler integration steps for each sample
        elif method == "digital_filter":
            self.prefilter = digitalFilter(L.F.num, L.F.den, P.Ts)
            self.control = digitalFilter(L.C.num, L.C.den, P.Ts)
        self.method = method

    def update(self, z_r, y_m):
        z = y_m[0][0]
        # prefilter
        if self.method == "state_space":
            self.updatePrefilterState(z_r)
            z_r_filtered = self.C_F @ self.x_F + self.D_F * z_r
        elif self.method == "digital_filter":
            z_r_filtered = self.prefilter.update(z_r)
        # error signal for longitudinal loop
        error = z_r_filtered - z
        # longitudinal controller
        if self.method == "state_space":
            self.updateControlState(error)
            F = self.C_C @ self.x_C + self.D_C * error
        elif self.method == "digital_filter":
            F = self.control.update(error)
        # if we use the saturation, it significantly affects performance,
        # for illustration purposes, we leave it as is. Try returning F.item(0)
        # directly to see the difference.
        force = saturate(F, P.F_max)
        return force

    def updatePrefilterState(self, z_r):
        for i in range(0, self.N):
            self.x_F = self.x_F + (P.Ts/self.N)*(
                self.A_F*self.x_F + self.B_F*z_r
            )

    def updateControlState(self, error_out):
        for i in range(0, self.N):
            self.x_C = self.x_C + (P.Ts/self.N)*(
                self.A_C*self.x_C + self.B_C*error_out
            )


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

