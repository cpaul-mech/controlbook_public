# Hummingbird Parameter File
import numpy as np

# Initial Conditions
phi0 = 0.0 * np.pi / 180  # roll angle in rads
theta0 = 0 * np.pi / 180  # pitch angle in rads
psi0 = 0.0 * np.pi / 180  # yaw angle in rads
phidot0 = 0.0              # roll rate in rads/sec
thetadot0 = 0.0         # pitch rate in rads/sec
psidot0 = 0.0              # yaw rate in rads/sec

# Initial Commanded PWM
left_pwm0 = 0.0
right_pwm0 = 0.0

# Physical parameters of the hummingbird known to the controller
g = 9.81
ell1 = 0.247
ell2 = -0.039
ell3x = -0.007
ell3y = -0.007
ell3z = 0.018
ellT = 0.355
d = 0.12
m1 = 0.108862
J1x = 0.000189
J1y = 0.001953
J1z = 0.001894
m2 = 0.4717
J2x = 0.00231
J2y = 0.003274
J2z = 0.003416
m3 = 0.1905
J3x = 0.0002222
J3y = 0.0001956
J3z = 0.000027

Fe = (m1 * ell1 * g + m2 * ell2 * g) / ellT
JT = m1 * ell1**2 + m2 * ell2**2 + J2z + m3 * (ell3x**2 + ell3y**2)
b_theta = ellT/(m1 * ell1**2 + m2 * ell2**2 + J1y + J2y)
b_psi = ellT * Fe / (JT + J1z)

# mixing matrix
unmixing = np.array([[1.0, 1.0], [d, -d]]) # converts fl and fr (LR) to force and torque (FT)
mixing = np.linalg.inv(unmixing) # converts force and torque (FT) to fl and fr (LR) 

# Simulation Parameters
t_start = 0.0  # Start time
Ts_Animation = 0.05
Ts_Controller = 0.01
Ts_GUI = 0.1

# saturation limits
force_max = 2.0                # Max force N
torque_max = 5.0                # Max torque, Nm

# Dictionary for communication
char_to_symbol = {'T': 'Theta', 'F': 'Phi', 'S': 'Psi', 'L': 'Left_PWM', 'R': 'Right_PWM'}
disp_dim = 6
animationDelay = 5.0
animationDelayCounter = 0.0
animationGoing = False

# Variables for H4
km = 0.338
km_min = 0.2
km_max = 0.5
km_step = 0.001

startMarker = 125
endMarker = 126
specialByte = 124

communicated = False
reading = False
bytesRec = 0
tempBuffer = ""

phi = 0.0
theta = 0.0
psi = 0.0

phi_ref = 0.0
theta_ref = 0.0
psi_ref = 0.0

# Variables for H7
theta_ref_min = -0.5
theta_ref_max = 0.5
theta_ref_step = 0.01

theta_kp = 0
theta_kp_min = 0
theta_kp_max = 2
theta_kp_step = 0.01

theta_ki = 0
theta_ki_min = 0
theta_ki_max = 1
theta_ki_step = 0.01

theta_kd = 0
theta_kd_min = 0
theta_kd_max = 1
theta_kd_step = 0.01

# From old code:
# kp_pitch = 0.98
# ki_pitch = 0.44
# kd_pitch = 0.58