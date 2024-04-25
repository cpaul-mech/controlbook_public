import numpy as np
import rodMassParam as PE

class controllerObsv:
    def __init__(self):
        self.x_hat = np.array([
            [0.0],  # estimate of theta
            [0.0],  # estimate of theta_hat
        ])
        self.tau_d1 = 0.0  # control torque, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = PE.K  # state feedback gain
        self.ki = PE.ki2  # Input gain
        self.L = PE.L  # observer gain
        self.A = PE.A  # system model
        self.B = PE.B
        self.C = PE.C
        self.limit = PE.tau_max
        self.tau_eq = PE.tau_eq
        self.Ts = PE.Ts  # sample rate of controller

    def update(self, theta_r, y):
        # update the observer and extract theta_hat
        x_hat = self.update_observer(y)
        theta_hat = x_hat[0][0]
        # integrate error
        error = theta_r - theta_hat
        self.integrator = self.integrator \
                          + (PE.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error
        # feedback linearizing torque tau_fl
        tau_fl = self.tau_eq
        # Compute the state feedback controller
        tau_tilde = -self.K @ x_hat - self.ki * self.integrator
        # compute total torque
        tau = saturate(tau_fl + tau_tilde[0], PE.tau_max)
        self.tau_d1 = tau
        return tau, x_hat

    def update_observer(self, y_m):
    # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + PE.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + PE.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + PE.Ts * F3, y_m)
        self.x_hat += PE.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y_m):
        # compute feedback linearizing torque tau_fl
        theta_hat = x_hat[0][0]
        tau_fl = self.tau_eq
        # xhatdot = A*(xhat-xe) + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat\
                   + self.B * (self.tau_d1 - tau_fl)\
                   + self.L * (y_m - self.C @ x_hat)
        return xhat_dot


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

