#%% [markdown]
## Laplace Transform and Transfer Function of the VTOL system
# Importing the necessary libraries and defining the symbols
from hw_04F_VTOL_EQ import *
#%% [markdown]
# Import the Laplace transform function
from sympy import laplace_transform
t, s = symbols('t s')
# Take the laplace transform of the state_variable_form
state_variable_form_laplace = laplace_transform(state_variable_form, t, s, noconds=True)
state_variable_form_laplace = simplify(state_variable_form_laplace)
print("State variable form in Laplace domain:")
display(Math(vlatex(state_variable_form_laplace)))
#%% [markdown]
# Define A, B, C, and D to find the TF of H/F
A = A_lin
print("A =")
display(Math(vlatex(A)))
B = B_lin
print("B =")
display(Math(vlatex(B)))
C = Matrix([[0.,0.,0.,1.,0.,0.]])
print("C =")
display(Math(vlatex(C)))
D = Matrix([[0.,0.]])
print("D =")
display(Math(vlatex(D)))

#%% [markdown]
# Define the Laplace variable and the transfer function
s = symbols('s')
n = A.shape[0]
G = C@((s*sp.eye(n)-A).inv())@B + D
G = simplify(G)
print("G =")
display(Math(vlatex(G)))


# %% [markdown]
# Calculat the TF for theta/tau
C1 = Matrix([[0.,0.,0.,0.,0.,1.]])
G_theta_tau = C1@((s*sp.eye(n)-A).inv())@B + D
G_theta_tau = simplify(G_theta_tau)
print("G_tau_theta =")
display(Math(vlatex(G_theta_tau)))
# %% [markdown]
# Calculate the TF for z/theta
C2 = Matrix([[0.,1.,0.,0.,0.,0.]])
G_z_theta = C2@((s*sp.eye(n)-A).inv())@B + D
G_z_theta = simplify(G_z_theta)
print("G_z_theta =")
display(Math(vlatex(G_z_theta)))
# %% [markdown]
# Solve HW problem F.6

long_state_variable_form = Matrix([[hdd_eom], [hd]])
long_states = Matrix([[hd], [h]])
long_inputs = Matrix([[F]])
lat_state_variable_form = Matrix([[zdd_eom], [zd], [thetadd_eom], [thetad]])
lat_states = Matrix([[zd], [z], [thetad], [theta]])
lat_inputs = Matrix([[tau]])
print("long_state_variable_form =")
display(Math(vlatex(long_state_variable_form)))
print("long_states =")
display(Math(vlatex(long_states)))
print("lat_state_variable_form =")
display(Math(vlatex(lat_state_variable_form)))
print("lat_states =")
display(Math(vlatex(lat_states)))
# %% [markdown]
# Solve for the A and B matrices of longitudinal dynamics
A_long = long_state_variable_form.jacobian(long_states)
B_long = long_state_variable_form.jacobian(long_inputs)
A_long_lin = simplify(A_long.subs([(hd,0.), (h,0.), (F, F_e)]))
B_long_lin = simplify(B_long.subs([(hd,0.), (h,0.), (F, F_e)]))
C_long = Matrix([[0., 1.]])
D_long = Matrix([[0.]])
print("A_long_lin =")
display(Math(vlatex(A_long_lin)))
print("B_long_lin =")
display(Math(vlatex(B_long_lin)))
print("C_long =")
display(Math(vlatex(C_long)))
print("D_long =")
display(Math(vlatex(D_long)))
# %% [markdown]
# Solve for the A and B matrices of lateral dynamics
A_lat = lat_state_variable_form.jacobian(lat_states)
B_lat = lat_state_variable_form.jacobian(lat_inputs)
A_lat_lin = simplify(A_lat.subs([(zd,0.), (z,0.), (thetad,0.), (theta,0.), (F, F_e), (tau, 0.)]))
B_lat_lin = simplify(B_lat.subs([(zd,0.), (z,0.), (thetad,0.), (theta,0.), (F, F_e), (tau, 0.)]))
C_lat = Matrix([[0., 1.,0.,0.],[0.,0.,0.,1.]])
D_lat = Matrix([[0.],[0.]])
print("A_lat_lin =")
display(Math(vlatex(A_lat_lin)))
print("B_lat_lin =")
display(Math(vlatex(B_lat_lin)))
print("C_lat =")
display(Math(vlatex(C_lat)))
print("D_lat =")
display(Math(vlatex(D_lat)))


# %%