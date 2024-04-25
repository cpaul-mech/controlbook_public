#%%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
import numpy as np
import massParam as P
import control as ctrl
from control import tf
#%% 
from sympy import sin, cos, diff, Matrix, symbols, Function, pretty_print, simplify, init_printing, latex
# define symbols
b, m, k1, k2, g, F = sp.symbols('b m k1 k2 g F')
s = sp.symbols('s')
b0 = 1/m
a1 = b/m
a0 = k1
k_p, k_d = sp.symbols('k_p k_d')
Z_Zr = ([b0*k_p],[1, a1+b0*k_d, a0+b0*k_p])
print("Z_Zr =")
display(Math(vlatex(Z_Zr)))
# %%
w_n = sp.sqrt(a0+b0*k_p)
print("w_n =")
display(Math(vlatex(w_n)))
zeta = sp.symbols('zeta')
k_d = (2.0*zeta*w_n - a1)/b0
print("k_d =")
display(Math(vlatex(k_d)))
# %%
