#%% [markdown]
## Assignment D.7
from hw05_DmassSpring_laplace import *
#%% [markdown]
# find the poles of G, the transfer function for part a
G_new = G[0]
G_new = 1/G_new
G_new = Matrix([[G_new]])
poles = sp.solve(G_new,s)
print("poles of open loop system =")
display(Math(vlatex(poles)))

# %% [markdown]
# Find the poles of G, with the block diagram equation for Z_r/Z
b0, k_p, k_d, a1, a0 = symbols('b0 k_p k_d a1 a0')
Y_Yr = (b0*k_p)/(s**2+ (a1 + b0*k_d)*s + a0 + b0*k_p)
print("Y/Y_r =")
display(Math(vlatex(Y_Yr)))
b0 = 1/m1
a1 = b/m1
a0 = k/m1
Z_Zr = (b0*k_p)/(s**2+ (a1 + b0*k_d)*s + a0 + b0*k_p)
print("Z/Z_r =")
display(Math(vlatex(Z_Zr)))

# %%
