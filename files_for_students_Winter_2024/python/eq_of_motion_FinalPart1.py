#%%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
import numpy as np
import massParam as P
#%% 
from sympy import sin, cos, diff, Matrix, symbols, Function, pretty_print, simplify, init_printing, latex
# define symbols
x1 = dynamicsymbols('x1')
x1dot = diff(x1)
m, F, g, t, c, k1, k2, b = sp.symbols('m F g t c k1 k2 b')
# define generalized coordinates and their derivatives
q = Matrix([[x1]])
qdot = q.diff()
#%% [markdown]
## Find kinetic energy (section 1.1)
# define position
p1 = Matrix([[0.0],[-x1], [0.0]])
v1 = diff(p1, t)
print("v1 =")
display(Math(vlatex(v1)))
KE = simplify(0.5*m*v1.T*v1)
KE = KE[0]
print("KE =")
display(Math(vlatex(KE)))
# %% [markdown]
## Define potential energy (section 1.2)
PE = k1*x1**2/2.0 + k2*x1**4/4.0 - m*g*x1
print("PE =")
display(Math(vlatex(PE)))

# %% [markdown]
## Define Lagrangian (section 1.3)
Lagrangian = simplify(KE - PE)
print("Lagrangian =")
display(Math(vlatex(Lagrangian)))

# %% [markdown]
## Define Generalized forces (section 1.4)
tau = F
damping_forces = -c*sp.sign(x1dot) - b*x1dot
RHS = tau + damping_forces
print("RHS =")
display(Math(vlatex(RHS)))
# %% [markdown]
## Derive the equations of motion using the Euler-lagrange expression (section 1.5)
print("dL/dq =")
display(Math(vlatex(diff(Lagrangian, q))))
print("dL/dqdot =")
display(Math(vlatex(diff(Lagrangian, qdot))))
print("d/dt(dL/dqdot) =")
display(Math(vlatex(diff(diff(Lagrangian, qdot), t))))
Left_side_EQ_Motion = simplify( diff(diff(Lagrangian, qdot), t) - diff(Lagrangian, q) )
Left_side_EQ_Motion = Left_side_EQ_Motion[0]
print("Left_side_EQ_Motion =")
display(Math(vlatex(Left_side_EQ_Motion)))

# %%
EOM = Left_side_EQ_Motion - RHS 
print("EOM =")
display(Math(vlatex(EOM)))
# %% 
x1ddot = sp.solve(EOM, diff(x1dot, t))
x1ddot = x1ddot[0]
print("x1ddot =")
display(Math(vlatex(x1ddot)))

# %%
