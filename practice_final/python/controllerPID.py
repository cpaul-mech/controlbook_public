import numpy as np
import rodMassParam as PE

class controllerPID:

    def __init__(self):
        self.kp = PE.kp
        self.ki = PE.ki
        self.kd = PE.kd
        self.limit = PE.tau_max
        self.beta = (2*PE.sigma-PE.Ts)/(2*PE.sigma+PE.Ts)
        self.Ts = PE.Ts
        self.theta_d1 = 0
        self.theta_dot = 0
        self.error_d1 = 0
        self.integrator = 0
        self.tau_eq = PE.tau_eq

    def update(self, theta_r, y):
        return tau

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







