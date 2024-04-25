import numpy as np
import rodMassParam as PE

class controllerPID:

    def __init__(self):
        self.kp = PE.kp
        self.ki = PE.ki1
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
        theta = y[0][0]
        error = theta_r - theta
        self.integrator = self.integrator \
            + (PE.Ts / 2) * (error + self.error_d1)
        # differentiate theta
        self.theta_dot = self.beta * self.theta_dot + (1 - self.beta) * ((theta - self.theta_d1) / PE.Ts)
        # PID control
        tau_tilde = self.kp * error \
            + self.ki * self.integrator \
                - self.kd * self.theta_dot
        tau_unsat = self.tau_eq + tau_tilde
        tau = saturate(tau_unsat, self.limit)

        if self.ki != 0.0:
            self.integrator = self.integrator \
                + PE.Ts / self.ki * (tau - tau_unsat)
        self.error_d1 = error
        self.theta_d1 = theta
        return tau

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u







