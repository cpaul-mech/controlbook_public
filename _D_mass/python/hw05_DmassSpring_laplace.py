#%% [markdown]
# Laplace Transform of the Mass-Spring-Damper System
# Importing the necessary libraries and defining the symbols
from hw04_DmassSpring_EQ import *

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
G = simplify(G)
print("G =")
display(Math(vlatex(G)))

# %%
