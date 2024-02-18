#%% [markdown]
## Laplace Transform and Transfer Function of the Block-Beam System
# Importing the necessary libraries and defining the symbols
from hw04_blockBeam_EQ import *
#%% [markdown]
# Import the Laplace transform function
from sympy import laplace_transform
t, s = symbols('t s')
# Take the laplace transform of the state_variable_form
state_variable_form_laplace = laplace_transform(state_variable_form, t, s, noconds=True)
print("State variable form in Laplace domain:")
display(Math(vlatex(state_variable_form_laplace)))
#%% [markdown]
# Define A, B, C, and D for the system
A = A_lin
print("A =")
display(Math(vlatex(A)))
B = B_lin
print("B =")
display(Math(vlatex(B)))
C = Matrix([[0., 1.,0.,0.],[0.,0.,0.,1.]])
print("C =")
display(Math(vlatex(C)))
D = Matrix([[0.],[0.]])
print("D =")
display(Math(vlatex(D)))

#%% [markdown]
# Define the Laplace variable and the transfer function
s = symbols('s')
n = A.shape[0]
G = C@(s*sp.eye(n)-A).inv()@B + D
G = simplify(G)
print("G =")
display(Math(vlatex(G)))

#%% [markdown]
# Solve for the z/theta transfer function
G_z_theta = G[0,0]*(G[1,0]**-1)
G_z_theta = simplify(G_z_theta)
print("G_z_theta =")
display(Math(vlatex(G_z_theta)))
# %% [markdown]
# remove the m1*g*z_e term from the transfer function
G_no_gravity = G.subs([(m1*g*z, 0)])
print("G_z_theta_no_gravity =")
display(Math(vlatex(G_no_gravity)))

# %%
