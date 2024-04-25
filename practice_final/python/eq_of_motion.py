#%%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
import numpy as np
import rodMassParam as P
#%% 
from sympy import sin, cos, diff, Matrix, symbols, Function, pretty_print, simplify, init_printing, latex
# define symbols
ell, J_cm, m, tau, g, t = sp.symbols('ell J_cm m tau g t')
theta = dynamicsymbols('theta')
thetadot = diff(theta)
# define generalized coordinates and their derivatives
q = Matrix([[theta]])
qdot = q.diff()

# define the kinetic energy
p1 = Matrix([[ell*cos(theta)],[ell*sin(theta)], [0]])
v1 = diff(p1, t)
omega = Matrix([[0], [0], [thetadot]])
J_cm = 0

# print v1
print("v1 =")
display(Math(vlatex(v1)))
# %% Solve for the kinetic energy
K = simplify(0.5*m*v1.T*v1 + 0.5*omega.T*J_cm*omega)
K = K[0]
print("K =")
display(Math(vlatex(K)))

# %% # define the potential energy
k1, k2 = sp.symbols('k1 k2')
PE = k1*theta**2/2 + k2*theta**4/4 + m*g*ell*sin(theta)
print("P =")
display(Math(vlatex(PE)))

# %%
L = simplify(K - PE)
print("L =")
display(Math(vlatex(L)))
# %% solve for left hand side of Euler lagrange equation
Left_side_EQ_Motion = simplify( diff(diff(L, qdot), t) - diff(L, q) )
Left_side_EQ_Motion = Left_side_EQ_Motion[0]
display(Math(vlatex(Left_side_EQ_Motion)))

# %% define the generalized forces
b = sp.symbols('b')
RHS = tau - b*thetadot
print("RHS =")
display(Math(vlatex(RHS)))
# %% Define full EOM
EOM = Left_side_EQ_Motion - RHS
print("EOM =")
display(Math(vlatex(EOM)))
thetaddot = sp.solve(EOM, diff(thetadot, t))
thetaddot = thetaddot[0]
print("thetaddot =")
display(Math(vlatex(thetaddot)))
# %% # now we need to solve for the linearized system
tau_e = sp.symbols('tau_e')
theta_e = sp.symbols('theta_e')
EOM_linearized = thetaddot.subs([(theta, theta_e), (thetadot, 0), (tau, tau_e)])
EOM_linearized = simplify(EOM_linearized)
print("EOM_linearized =")
display(Math(vlatex(EOM_linearized)))
# %%
tau_e = (sp.solve(EOM_linearized, tau_e))
tau_e = tau_e[0]
tau_e = simplify(tau_e)
print("tau_e =")
display(Math(vlatex(tau_e)))

# %% # Now we need to linearize the model found in part 1 around the theta_e, tau_e
state_variable_form = Matrix([[thetadot], [thetaddot]])
states = Matrix([[theta], [thetadot]])
inputs = Matrix([tau])
print("state_variable_form =")
display(Math(vlatex(state_variable_form)))
print("states =")
display(Math(vlatex(states)))
print("inputs =")
display(Math(vlatex(inputs)))
# %%
A = state_variable_form.jacobian(states)
print("A =")
display(Math(vlatex(A)))
B = state_variable_form.jacobian(inputs)
theta_e = 0.0
A_lin = simplify(A.subs([(thetadot, 0.0), (theta, 0.0), (tau, P.tau_eq)]))
B_lin = simplify(B.subs([(thetadot, 0.0), (theta, 0.0), (tau, P.tau_eq)]))
print("A_lin =")
display(Math(vlatex(A_lin)))
print("B_lin =")
display(Math(vlatex(B_lin)))
# %%
C = Matrix([[1, 0]])
D = Matrix([[0]])
# %% 
s = sp.symbols('s')
transfer_f = C @ ((s*sp.eye(2,2)-A_lin).inv()) @ B_lin + D
transfer_f = simplify(transfer_f)
print("Transfer function =")
display(Math(vlatex(transfer_f)))
Plant = transfer_f[0]
# %%
Theta, Theta_r, k_p, k_d, s = sp.symbols('Theta Theta_r k_p k_d s')
eq1 = sp.Eq(Theta, Plant*(k_p*(Theta_r-Theta) - k_d*s*Theta))

print("eq1 =")
display(Math(vlatex(eq1)))
# Solve for Theta,
k_p_act = 3.0/(20.*(np.pi/180.))
Theta = sp.solve(eq1, Theta)
Theta = Theta[0]
Theta = Theta.subs([(ell, P.ell), (m, P.m), (b, P.b),(k1, P.k1)])
Theta = simplify(Theta)
Theta = Theta/Theta_r
print("Theta =")
display(Math(vlatex(Theta)))
# %%
w_n = np.sqrt(160.0*(0.02 + k_p_act))
zeta = 0.707
eq2 = sp.Eq(2*zeta*w_n, 160.0*(0.1 + k_d))
print("eq2 =")
display(Math(vlatex(eq2)))
k_d = sp.solve(eq2, k_d)
k_d = k_d[0]
k_d = simplify(k_d)
print("k_d =")
display(Math(vlatex(k_d)))

# %% now display the roots of the characteristic equation
char_eq = sp.Eq(s**2 + 2*zeta*w_n*s + w_n**2, 0)
