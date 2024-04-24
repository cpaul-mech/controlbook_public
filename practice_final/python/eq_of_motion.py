#%%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
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
P = 0.5*k1*theta + 0.25*k2*theta**4 + m*g*ell*sin(theta)
print("P =")
display(Math(vlatex(P)))

# %%
L = simplify(K - P)
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

# %%
