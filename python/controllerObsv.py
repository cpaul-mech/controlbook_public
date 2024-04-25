import numpy as np
import massParam as P

class controllerObsv:
    def __init__(self):
        self.xhat = np.array([[0.0], [0.0]])
        self.F_d1 =0.0
        self.integrator = 0.0
        self.error_d1 = 0.0
        self.K = P.K
        self.ki = P.ki2
        self.L = P.L
        self.A = P.A
        self.B = P.B
        self.C = P.C
        self.limit = P.F_max
        self.F_e = P.F_eq
        self.Ts = P.Ts

    def update(self, z_r, y):
        # update the observer and extract theta_hat
        x_hat = self.update_observer(y)
        theta_hat = x_hat[0][0]
        # integrate error
        error = theta_r - theta_hat
        self.integrator = self.integrator \
                          + (PE.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error 
        F_fl = self.F_e
        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat - self.ki * self.integrator
        # compute total torque
        F = saturate(F_fl + tau_tilde[0], PE.tau_max)
        self.tau_d1 = tau
        return F, x_hat

    def update_observer(self, y):
        return x_hat

    def observer_f(self, x_hat, y):
        return xhat_dot

    def integrateError(self, error):

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

