#%% [markdown]
## Assignment D.7
from hw05_DmassSpring_laplace import *
#%% [markdown]
## Part A
# find the poles of G, the transfer function for part a
import massParam as P
G_new = G.subs([(m1,P.m), (b,P.b), (k,P.k)])
G_new = G_new[0]
G_new = 1/G_new
G_new = Matrix([[G_new]])
poles = sp.solve(G_new,s)
print("poles of open loop system =")
display(Math(vlatex(poles)))

# %% [markdown]
## Part B
# Find the poles of G, with the block diagram equation for Z_r/Z
b0, k_p, k_d, a1, a0 = symbols('b0 k_p k_d a1 a0')
Y_Yr = (b0*k_p)/(s**2+ (a1 + b0*k_d)*s + a0 + b0*k_p)
print("From the in-class notes: Y/Y_r =")
display(Math(vlatex(Y_Yr)))
b0 = 1/m1
a1 = b/m1
a0 = k/m1
Z_Zr = (b0*k_p)/(s**2+ (a1 + b0*k_d)*s + a0 + b0*k_p)
print("Subsituting b0, a0, and a1: Z/Z_r =")
display(Math(vlatex(Z_Zr)))

# %% [markdown]
# Now solve for the poles of Z/Z_r to find the closed loop poles
characteristic_eq = Z_Zr *(m1/k_p)
characteristic_eq = 1/characteristic_eq
characteristic_eq = simplify(characteristic_eq)
poles_part_b = sp.solve(characteristic_eq,s)
print("poles of closed loop system =")
display(Math(vlatex(poles_part_b)))
# %% [markdown]
## Part C
# set these poles equal to the desired poles and solve for the gains k_p and k_d
desired_poles = [-1.5, -1]
pole1 = poles_part_b[0]
pole2 = poles_part_b[1]

k_d_eq = pole1 - desired_poles[0]
k_d_eq = sp.solve(k_d_eq, k_d)
k_d_eq = k_d_eq[0]
print("k_d =")
display(Math(vlatex(k_d_eq)))

k_p_eq = pole2 - desired_poles[1]
k_p_eq = k_p_eq.subs(k_d,k_d_eq)
k_p_eq = simplify(k_p_eq)
#%%
k_p_eq = sp.solve(k_p_eq, k_p)
k_p_eq = k_p_eq[0]
print("k_p =")
display(Math(vlatex(k_p_eq)))
# %% [markdown]
# Re-solve for k_d using the new k_p
k_d_eq = k_d_eq.subs(k_p,k_p_eq)
print("k_d =")
display(Math(vlatex(k_d_eq)))
# %% [markdown]
## Subsitute in values from massparam file
import massParam as P
k_p_eq = k_p_eq.subs([(m1, P.m), (k, P.k)])
k_d_eq = k_d_eq.subs([(m1, P.m), (b, P.b)])
print("k_p =")
display(Math(vlatex(k_p_eq)))
print("k_d =")
display(Math(vlatex(k_d_eq)))


# %%
