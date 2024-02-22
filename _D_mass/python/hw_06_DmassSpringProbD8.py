#%% [markdown]
## Problem D.8
from hw06_DmassSpring import *
import numpy as np
# %% [markdown]
## Part A
# Solve for zeta with the given zeta and rise time.
zeta = 0.7
omega_n = symbols('omega_n')
equation = 0.5*(sp.pi/(omega_n*sp.sqrt(1-zeta**2))) - 2.
omega_n = sp.solve(equation, omega_n)
print("omega_n =")
display(Math(vlatex(omega_n)))
# %% [markdown]
# find characteristic equation
ce_eq = sp.Eq(s**2 + 2*zeta*omega_n[0]*s + omega_n[0]**2, 0)
print("Characteristic equation =")
display(Math(vlatex(ce_eq)))
print("The poles are located at ")
poles = sp.solve(ce_eq,s)
display(Math(vlatex(poles)))
# %% [markdown]
# Now solve for the gains k_p and k_d
eq1 = sp.Eq(omega_n[0]**2, a0 + b0*k_p)
print("Equation for first coefficient = ")
display(Math(vlatex(eq1)))
k_p_act = eq1.subs([(m1, P.m), (k, P.k)])
k_p_act = sp.solve(k_p_act, k_p)
k_p_act = k_p_act[0]
print("k_p =")
display(Math(vlatex(k_p_act)))
print("Equation for second coefficient = ")
eq2 = sp.Eq(2*zeta*omega_n[0], a1 + b0*k_d)
display(Math(vlatex(eq2)))
k_d_act = eq2.subs([(m1, P.m), (b, P.b)])
k_d_act = sp.solve(k_d_act, k_d)
k_d_act = k_d_act[0]
print("k_d =")
display(Math(vlatex(k_d_act)))

# %% 
