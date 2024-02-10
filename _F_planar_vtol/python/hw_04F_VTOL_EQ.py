#%%
from hw03_EQ_motion_calcF import *


#%%
############################################################
### Defining vectors for x_dot, x, and u, then taking partial derivatives
############################################################


# defining derivative of states, states, and inputs symbolically
### for this first one, keep in mind that zdd_eom is actually a full
### row of equations, while zdd is just the symbolic variable itself. 
state_variable_form = Matrix([[zdd_eom], [zd], [hdd_eom], [hd], [thetadd_eom], [thetad]])
states = Matrix([[zd], [z], [hd], [h], [thetad], [theta]])
inputs = Matrix([[F], [tau]])
print("state_variable_form =")
display(Math(vlatex(state_variable_form)))
print("states =")
display(Math(vlatex(states)))
print("inputs =")
display(Math(vlatex(inputs)))

#%% [markdown]
# Set all the derivatives to zero and solve for the equilibrium points.
z_e, h_e, theta_e, F_e, tau_e = symbols('z_e h_e theta_e F_e tau_e')
equilibrium_system = state_variable_form.subs([(zdd,0.), (zd,0.), (z, z_e), (hdd, 0.), (hd, 0.), (h, h_e), (thetadd, 0.), (thetad, 0.), (theta, theta_e), (F, F_e), (tau, tau_e)])
print("Equilibrium system: ")
display(Math(vlatex(equilibrium_system)))
equilibrium_system = equilibrium_system.subs(theta_e, 0.)
print("Equilibrium system with theta_e = 0: ")
display(Math(vlatex(equilibrium_system)))
equilibrium_points = sp.solve(equilibrium_system, (F_e, tau_e))
print("Equilibrium solved for F_e & tau_e: ", equilibrium_points)
display(Math(vlatex(equilibrium_points)))
#%%
F_e = equilibrium_points[F_e]
tau_e = equilibrium_points[tau_e]

#%%
# finding the jacobian with respect to states (A) and inputs (B)
A = state_variable_form.jacobian(states)
B = state_variable_form.jacobian(inputs)

# sub in values for equilibrium points 
A_lin = simplify(A.subs([(zd,0.), (z,0.), (hd,0.), (h,0.), (thetad,0.), (theta,0.), (F, F_e), (tau, 0.)]))
B_lin = simplify(B.subs([(zd,0.), (z,0.), (hd,0.), (h,0.), (thetad,0.), (theta,0.), (F, F_e), (tau, 0.)]))
print("A_lin =")
display(Math(vlatex(A_lin)))
print("B_lin =")
display(Math(vlatex(B_lin)))


# %%