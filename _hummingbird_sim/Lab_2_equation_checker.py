#%%
# Use this program to check my equations used in hummingbirdDynamics.py

import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
from sympy import sin, cos, diff, Matrix, symbols, Function, pretty_print, simplify, init_printing, latex

#%%
# Define the symbols and variables
m1, m2, m3, ell1, ell2, ell3x, ell3y, ell3z, J1x, J1y, J1z, J2x, J2y, J2z, J3x, J3y, J3z, phi, theta, sci = sp.symbols('m1 m2 m3 ell1 ell2 ell3x ell3y ell3z J1x J1y J1z J2x J2y J2z J3x J3y J3z, phi, theta, sci')
M22 = m1 * ell1**2 + m2*ell2**2 + J2y + J1y*cos(phi)**2 + J1z*sin(phi)**2
M23 = (J1z - J1y)*sin(phi)*cos(phi)*cos(theta)
M33 = (m1*ell1**2 + m2*ell2**2 + J2z + J1y*sin(phi)**2 + J1z*cos(phi)**2)*cos(theta)**2 + (J1x + J2x)*sin(theta)**2 + m3*(ell3x**2 + ell3y**2) + J3z
print("M22 =")
display(Math(vlatex(M22)))
print("M23 =")
display(Math(vlatex(M23)))
print("M33 =")
display(Math(vlatex(M33)))

#%%
theta = dynamicsymbols('theta')
thetadot = theta.diff()
thetadotdot = thetadot.diff()
phi = dynamicsymbols('phi')
phidot = phi.diff()
phidotdot = phidot.diff()
psi = dynamicsymbols('psi')
psidot = psi.diff()
psidotdot = psidot.diff()

# Create the missing Symbols 
g, pwm_left, pwm_right, ellT, d, km, M22, M23, M33 = sp.symbols('g pwm_left pwm_right ellT d km M22 M23 M33')
M = simplify(Matrix([[J1x, 0, -J1x*sin(theta)],
                      [0, M22, M23],
                      [-J1x*sin(theta), M23, M33]
                      ]))
C = simplify(Matrix([[(J1y - J1z)*sin(phi)*cos(phi)*(thetadot**2 - cos(theta)**2*psidot**2) + ((J1y - J1z)*(cos(phi)**2 - sin(phi)**2) - J1x)*cos(theta)*thetadot*psidot],
                [2.0*(J1z - J1y)*sin(phi)*cos(phi)*phidot*thetadot + ((J1y - J1z)*(cos(phi)**2 - sin(phi)**2) + J1x)*cos(theta)*phidot*psidot - 0.5*(2.0*(J1x + J2x - m1*ell1**2-m2*ell2**2 - J2z - J1y*sin(phi)**2 - J1z*cos(phi)**2)*sin(theta)*cos(theta))*phidot**2],
                [thetadot**2*(J1z - J1y)*sin(phi)*cos(phi)*sin(theta) + ((J1y - J1z)*(cos(phi)**2 - sin(phi)**2) - J1x)*cos(theta)*phidot*psidot + (J1z - J1y)*sin(phi)*cos(phi)*sin(theta)*thetadot**2 + 2.0*(J1y - J1z)*sin(phi)*cos(phi)*phidot*psidot +
                2.0*(-m1*ell1**2 - m2*ell2**2 - J2z + J1x + J2x + J1y*sin(phi)**2 + J1z*sin(phi)**2)*sin(theta)*cos(theta)*thetadot*psidot],
                ]))
partialP = simplify(Matrix([[0.0],
                        [(m1*ell1 + m2*ell2)*g*cos(theta)],
                        [0.0],
                    ]))
force = km * (pwm_left + pwm_right)
torque = d * km * (pwm_left - pwm_right) # what are the actual forces? seems like we'll need to convert them at some point. 
tau = simplify(Matrix([[d*(pwm_left - pwm_right)],
                [ellT*(pwm_left + pwm_right)*cos(phi)],
                [ellT*(pwm_left + pwm_right)*cos(theta)*sin(phi) - d*(pwm_left - pwm_right)*sin(theta)]]))
# %%
print("M =")
display(Math(vlatex(M)))
print("C =")
display(Math(vlatex(C)))
print("partialP =")
display(Math(vlatex(partialP)))
print("force =")
display(Math(vlatex(force)))
print("torque =")
display(Math(vlatex(torque)))
print("tau =")
display(Math(vlatex(tau)))
#%%