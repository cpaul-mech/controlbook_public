#%% [markdown]
## Laplace Transform and Transfer Function of the Mass-Spring-Damper System
# Importing the necessary libraries and defining the symbols
from hw04_DmassSpring_EQ import *
#%% [markdown]
# Import the Laplace transform function
from sympy import laplace_transform
t, s = symbols('t s')
# Take the laplace transform of the state_variable_form
print("full_eom =")
display(Math(vlatex(full_eom)))
full_eom_laplace = laplace_transform(full_eom, t, s, noconds=True)
print("full_eom_laplace =")
display(Math(vlatex(full_eom_laplace)))

#%% [markdown]
# Define A, B, C, and D for the system
A = A_lin
print("A =")
display(Math(vlatex(A)))
B = B_lin
print("B =")
display(Math(vlatex(B)))
C = Matrix([[0., 1.]])
print("C =")
display(Math(vlatex(C)))
D = Matrix([[0.]])
print("D =")
display(Math(vlatex(D)))

#%% [markdown]
# Define the Laplace variable and the transfer function
s = symbols('s')
G = C*(s*sp.eye(2)-A).inv()*B + D
G = (G*((1/m1)/(1/m1)))
print(" =")
display(Math(vlatex(G)))

# %%
