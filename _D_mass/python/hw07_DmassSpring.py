#%% [markdown]
## in this homework we will be simplifying some symbolic equations
# Problem D.9
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
from sympy import sin, cos, diff, Matrix, symbols, Function, pretty_print, simplify, init_printing, latex
# %% [markdown]
k_p, k_d, k_I, s, = symbols('k_p k_d k_I s')
P_of_S = 0.2/(s**2 + 0.1*s + 0.6)
C_of_S = k_p + k_d*s + k_I/s
# %% [markdown]
# The transfer function of the system is given by:
H_new = P_of_S*C_of_S/(1 + P_of_S*C_of_S)
H_new = simplify(H_new)
print("H_new = ")
display(Math(vlatex(H_new)))
# %% [markdown]
# Part B: TF with D_in as new input
C_of_S = k_p + k_d*s
print("DF is given by P/(1+PC)")
part_B = P_of_S/(1 + P_of_S*C_of_S)
part_B = simplify(part_B)
print("new_TF = ")
display(Math(vlatex(part_B)))
# %% [markdown]
## Problem E.9:
C = k_p + k_d*s
ell, m2, m1, Z_e = symbols('ell m2 m1 Z_e')
P = (ell/(m2*ell**2/3 + m1*Z_e**2))/s**2
H = P*C/(1 + P*C)
H = simplify(H)
print("pd control for inner loop tf = ")
display(Math(vlatex(H)))

# %% [markdown]
# with respect to a new input D_in
H_2 = P/(1 + P*C)
H_2 = simplify(H_2)
print("new input tf = ")
display(Math(vlatex(H_2)))

# %% [markdown]
# Part B: repeat part a but P(s) = -g/s**2 for outer loop
g= symbols('g')
P = -g/s**2
H = P*C/(1 + P*C)
H = simplify(H)
print("pd control for outer loop tf = ")
display(Math(vlatex(H)))
# %% [markdown]
# Now add an integrator to the outer loop
C = k_p + k_d*s + k_I/s
H = P*C/(1 + P*C)
H = simplify(H)
print("PID control for outer loop tf = ")
display(Math(vlatex(H)))

# %%
# Again we see if taking the TF with respect to a new input D_in changes anything
H_2 = P/(1 + P*C)
H_2 = simplify(H_2)
print("new input tf = ")
display(Math(vlatex(H_2)))
# %%
