#%% [markdown]
## H.9 worked out:
# Part 1
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display
import hummingbirdParam as P
#%%
J1x, s, k_p, k_d = sp.symbols('J1x s k_p k_d')
P = (1/J1x)/s**2
C = (k_d*s + k_p)
G = P*C
G = sp.simplify(G)
print("G =")
display(Math(vlatex(G)))
# %%
E_R = 1/(1+G)
E_R = sp.simplify(E_R)
print("E_R =")
display(Math(vlatex(E_R)))
print("Steady state error for Reference input:\n")
print("unit step input =", sp.limit(s*E_R*(1/s),s, 0))
print("unit ramp input =", sp.limit(s*E_R*(1/s**2),s, 0))
print("unit parabolic input =", sp.limit(s*E_R*(1/s**3),s, 0))
print("Thus the system type is 2")

# %%
E_D = P/(1+P*C)
E_D = sp.simplify(E_D)
print("E_D =")
display(Math(vlatex(E_D)))
print("Steady state error for disturbance input:\n")
print("unit step input =", sp.limit(s*E_D*(1/s),s, 0))
print("unit ramp input =", sp.limit(s*E_D*(1/s**2),s, 0))
print("unit parabolic input =", sp.limit(s*E_D*(1/s**3),s, 0))
print("thus the system type is 0")
# %% [markdown]
# Part 2:
b_psi = sp.symbols('b_psi')
P_psi = (b_psi)/s**2 
C_psi = (k_d*s + k_p)
print("PC = ")
PC = P_psi*C_psi
display(Math(vlatex(sp.simplify(PC))))
# %%
E_R_psi = 1/(1+PC)
E_R_psi = sp.simplify(E_R_psi)
print("E_R_psi =")
display(Math(vlatex(E_R_psi)))
print("Steady state error for Reference input:\n")
print("unit step input =", sp.limit(s*E_R_psi*(1/s),s, 0))
print("unit ramp input =", sp.limit(s*E_R_psi*(1/s**2),s, 0))
print("unit parabolic input =", sp.limit(s*E_R_psi*(1/s**3),s, 0))
print("Thus the system type is 2")
#%%
D_R_psi = P_psi/(1+P_psi*C_psi)
D_R_psi = sp.simplify(D_R_psi)
print("D_R_psi =")
display(Math(vlatex(D_R_psi)))
print("Steady state error for disturbance input:\n")
print("unit step input =", sp.limit(s*D_R_psi*(1/s),s, 0))
print("unit ramp input =", sp.limit(s*D_R_psi*(1/s**2),s, 0))
print("unit parabolic input =", sp.limit(s*D_R_psi*(1/s**3),s, 0))
print("thus the system type is 0")
# %% [markdown]
# Part 3:
b_theta = sp.symbols('b_theta')
P_theta = (b_theta)/s**2
C_theta = (k_d*s + k_p)
print("PC = ")
PC_theta = P_theta*C_theta
display(Math(vlatex(sp.simplify(PC_theta))))
# %%
E_R_theta = 1/(1+PC_theta)
E_R_theta = sp.simplify(E_R_theta)
print("E_R_theta =")
display(Math(vlatex(E_R_theta)))
print("Steady state error for Reference input:\n")
print("unit step input =", sp.limit(s*E_R_theta*(1/s),s, 0))
print("unit ramp input =", sp.limit(s*E_R_theta*(1/s**2),s, 0))
print("unit parabolic input =", sp.limit(s*E_R_theta*(1/s**3),s, 0))
print("Thus the system type is 2")
#%%
D_R_theta = P_theta/(1+P_theta*C_theta)
D_R_theta = sp.simplify(D_R_theta)
print("D_R_theta =")
display(Math(vlatex(D_R_theta)))
print("Steady state error for disturbance input:\n")
print("unit step input =", sp.limit(s*D_R_theta*(1/s),s, 0))
print("unit ramp input =", sp.limit(s*D_R_theta*(1/s**2),s, 0))
print("unit parabolic input =", sp.limit(s*D_R_theta*(1/s**3),s, 0))
print("thus the system type is 0")
#%%