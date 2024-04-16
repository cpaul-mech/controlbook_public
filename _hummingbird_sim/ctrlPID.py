import numpy as np
import hummingbirdParam as P


class ctrlPID:
    def __init__(self): #theta for vtol = phi for hummingbird
        # tuning parameters
        # --------------------- PITCH OR THETA CONTROL ---------------------
        tr_theta = 0.5 # rise time for pitch control, I specify this and guess.
        zeta_theta = 0.95
        self.ki_theta = 0.75 # 
        self.ki_thetadot_limit = 5.0*np.pi/180.0
        b_theta = P.ellT/(P.m1 * P.ell1**2 + P.m2 * P.ell2**2 + P.J1y + P.J2y)
        wn_theta = np.pi/(2.0*tr_theta*np.sqrt(1-zeta_theta**2))
        self.kp_theta = wn_theta**2/b_theta
        self.kd_theta = 2.*zeta_theta*wn_theta/b_theta
        # --------------------- ROLL OR PHI CONTROL ---------------------
        tr_phi = 0.1 ## ROLL CONTROL INNER LOOP!!
        zeta_phi = 0.707
        wn_phi = np.pi/(2.*tr_phi*np.sqrt(1-zeta_phi**2))
        self.kp_phi = wn_phi**2*P.J1x
        self.kd_phi = 2.*zeta_phi*wn_phi*P.J1x
        # --------------------- YAW OR PSI CONTROL ---------------------
        M = 10.0  # time separation between inner and outer lateral loops
        tr_psi = tr_phi*M # YAW CONTROL
        zeta_psi = 0.73
        self.ki_psi = 0.25
        self.ki_psidot_limit = 5.0*np.pi/180.0
        F_e = (P.m1*P.ell1 + P.m2*P.ell2)*P.g/ P.ellT
        J_T = P.m1*P.ell1**2 + P.m2*P.ell2**2 + P.J2z + P.m3*(P.ell3x**2 + P.ell3y**2)
        b_psi = P.ellT*F_e/(J_T + P.J1z)
        wn_psi = np.pi/(2.*tr_psi*np.sqrt(1-zeta_psi**2))
        self.kp_psi = wn_psi**2/b_psi
        self.kd_psi = 2.*zeta_psi*wn_psi/b_psi
        
        
        # print gains to terminal
        print('kp_theta: ', self.kp_theta)
        print('ki_theta: ', self.ki_theta)
        print('kd_pitch: ', self.kd_theta) 
        print('kp_psi: ', self.kp_psi)
        print('ki_psi: ', self.ki_psi)
        print('kd_psi: ', self.kd_psi)
        print('kp_phi: ', self.kp_phi)
        print('kd_phi: ', self.kd_phi)
        # sample rate of the controller
        self.Ts = P.Ts
        # dirty derivative parameters
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - self.Ts) / (2 * sigma + self.Ts)
        # delayed variables
        self.theta_d1 = 0. #PITCH
        self.theta_dot = 0.
        self.integrator_theta = 0.
        self.error_theta_d1 = 0.  # pitch error delayed by 1
        self.phi_d1 = 0. #ROLL
        self.phi_dot = 0.
        self.integrator_phi = 0.
        self.error_phi_d1 = 0.
        self.psi_d1 = 0. #YAW
        self.psi_dot = 0.
        self.integrator_psi = 0.
        self.error_psi_d1 = 0.


    def update(self, r, y): #r = np.array([[theta_ref], [psi_ref]])  y = np.array([[phi], [theta], [psi]])
        theta_ref = r[0][0]
        psi_ref = r[1][0]
        phi = y[0][0]
        theta = y[1][0]
        psi = y[2][0]
        force_fl = (P.m1*P.ell1 + P.m2*P.ell2)*P.g*np.cos(theta)/P.ellT
        # compute errors
        error_theta = theta_ref - theta 
        error_psi = psi_ref - psi
        # update differentiators
        self.theta_dot = self.beta*self.theta_dot + (1-self.beta)*((theta - self.theta_d1) / self.Ts) # dirty derivative, figure out where this comes from.
        self.psi_dot = self.beta*self.psi_dot + (1-self.beta)*((psi - self.psi_d1) / self.Ts) # dirty derivative, figure out where this comes from.
        self.phi_dot = self.beta*self.phi_dot + (1-self.beta)*((phi - self.phi_d1) / self.Ts) # dirty derivative, figure out where this comes from.
        
        # update integrators
        if abs(self.theta_dot) < self.ki_thetadot_limit:
            self.integrator_theta = self.integrator_theta + (self.Ts/2.0)*(error_theta + self.error_theta_d1)
        if abs(self.psi_dot) < self.ki_psidot_limit:
            self.integrator_psi = self.integrator_psi + (self.Ts/2.0)*(error_psi + self.error_psi_d1)

        # pitch control
        f_tilde = self.kp_theta*error_theta - self.kd_theta*self.theta_dot + self.ki_theta*self.integrator_theta
        force_unsat = force_fl + f_tilde
        force = saturate(force_unsat, -P.force_max, P.force_max)
        
        # roll control
        phi_ref = self.kp_psi*error_psi - self.kd_psi*self.psi_dot + self.ki_psi*self.integrator_psi
        error_phi = phi_ref - phi
        tau = self.kp_phi*error_phi - self.kd_phi*self.phi_dot
        torque = saturate(tau, -P.torque_max, P.torque_max)

        # convert force and torque to pwm signals
        pwm = np.array([[force + torque / P.d],               # u_left
                      [force - torque / P.d]]) / (2.0 * P.km)   # r_right          
        pwm = saturate(pwm, 0, 1)
        # update all delayed variables
        self.theta_d1 = theta
        self.error_theta_d1 = error_theta
        self.phi_d1 = phi
        self.error_phi_d1 = error_phi
        self.psi_d1 = psi
        self.error_psi_d1 = error_psi
    
        # return pwm plus reference signals
        return pwm, np.array([[phi_ref], [theta_ref], [psi_ref]])


def saturate(u, low_limit, up_limit):
    if isinstance(u, float) is True:
        if u > up_limit:
            u = up_limit
        if u < low_limit:
            u = low_limit
    else:
        for i in range(0, u.shape[0]):
            if u[i][0] > up_limit:
                u[i][0] = up_limit
            if u[i][0] < low_limit:
                u[i][0] = low_limit
    return u




