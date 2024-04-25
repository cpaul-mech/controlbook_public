import numpy as np
import massParam as P

class controllerPID:

    def __init__(self):
        self.kp = P.k_p
        self.ki = P.k_i1
        self.kd = P.k_d
        self.limit = P.F_max
        self.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts) # Don't know where this comes from
        self.Ts = P.Ts
        self.z_d1 = 0.0
        self.z_dot = 0.0
        self.error_d1 = 0.0 
        self.integrator = 0.0
        self.F_e = P.F_eq

    def update(self, z_r, y):
        z = y[0][0]
        error = z_r - z
        self.integrator = self.integrator \
            + (P.Ts / 2) * (error + self.error_d1)
        # differentiate theta
        self.z_dot = self.beta * self.z_dot + (1 - self.beta) * ((z - self.z_d1) / P.Ts)
        # PID control
        F_tilde = self.kp * error \
            + self.ki * self.integrator \
                - self.kd * self.z_dot
        F_unsat = self.F_e + F_tilde
        F = saturate(F_unsat, self.limit)
        if self.ki != 0.0:
            self.integrator = self.integrator \
                + P.Ts / self.ki * (F - F_unsat)
        self.error_d1 = error
        self.theta_d1 = z
        return F

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u







