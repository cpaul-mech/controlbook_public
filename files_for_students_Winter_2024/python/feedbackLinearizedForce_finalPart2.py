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
z = dynamicsymbols('z')
zdot = diff(z)
zddot=diff(zdot)
m, k1, k2, g, F, b = sp.symbols('m k1 k2 g F b')
RHS = F - b*zdot
LHS = m*zddot + k1*z + k2*z**3 - m*g/sp.sqrt(2)
EOM = LHS - RHS
zddot = sp.solve(EOM, zddot)
zddot = zddot[0]
print("EOM =")
display(Math(vlatex(EOM)))
# %% [markdown]
## Find the equilibrium force required to hold mass at point z_e
z_e = sp.symbols('z_e')
F_e = sp.symbols('F_e')
EOM_lin = EOM.subs([(F, F_e), (z, z_e), (zdot, 0), (zddot, 0)])
EOM_lin = simplify(EOM_lin)
print("EOM_lin =")
display(Math(vlatex(EOM_lin)))
F_e_eq = sp.solve(EOM_lin, F_e)
F_e_eq = F_e_eq[0]
print("F_e_eq =")
display(Math(vlatex(F_e_eq)))
# %% 
xdot = Matrix([[zdot], [zddot]])
x = Matrix([[z], [zdot]])
u = Matrix([[F]])
A = xdot.jacobian(x)
B = xdot.jacobian(u)
C = Matrix([[1.0, 0.0]])
D = Matrix([[0.0]])
print("A =")
display(Math(vlatex(A)))
print("B =")
display(Math(vlatex(B)))
z_e = 0.0
A_lin = simplify(A.subs([(zdot, 0.0), (z, z_e), (F, F_e_eq)]))
B_lin = simplify(B.subs([(zdot, 0.0), (z, z_e), (F, F_e_eq)]))
print("A_lin =")
display(Math(vlatex(A_lin)))
print("B_lin =")
display(Math(vlatex(B_lin)))

lin_eom = A_lin*x.subs(z, z_e) + B_lin*u
print("Linearized EOM =")
display(Math(vlatex(lin_eom)))
s = sp.symbols('s')
transferf = C @ ((s*sp.eye(2)-A_lin).inv()) @ B_lin+D
print("Transfer Function =")
display(Math(vlatex(transferf)))
# %%
