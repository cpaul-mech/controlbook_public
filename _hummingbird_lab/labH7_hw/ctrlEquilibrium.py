import numpy as np
import hummingbirdParam as P

class ctrlEquilibrium:
    def __init__(self):
        pass 

    def update(self, r, y):
        km = P.km          
        force_equilibrium = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) / P.ellT
        force = force_equilibrium
        torque = 0.
        pwm = np.array([[force + torque / P.d],
                      [force - torque / P.d]]) / (2 * km)
        pwm = saturate(pwm, 0, 1)
        return pwm

def saturate(u, low_limit, up_limit):
    if isinstance(u, float) is True:
        u = np.max((np.min((u, up_limit)), low_limit))
    else:
        for i in range(0, u.shape[0]):
            u[i][0] = np.max((np.min((u[i][0], up_limit)), low_limit))
    return u




