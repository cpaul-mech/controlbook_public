#%%
from hw03_EQ_motion_calcF import *


#%%
############################################################
### Defining vectors for x_dot, x, and u, then taking partial derivatives
############################################################


# defining derivative of states, states, and inputs symbolically
### for this first one, keep in mind that zdd_eom is actually a full
### row of equations, while zdd is just the symbolic variable itself. 
state_variable_form = Matrix([[zdd_eom], [zd]])
states = Matrix([[zd], [z]])
inputs = Matrix([[F]])

#%% [markdown]
# Set all the derivatives to zero and solve for the equilibrium points.
F_e, z_e = symbols('F_e, z_e')
equilibrium_system = state_variable_form.subs([(zdd,0.), (zd,0.),(F, F_e), (z, z_e)])
print("Equilibrium system: ")
display(Math(vlatex(equilibrium_system)))
equilibrium_points = sp.solve(equilibrium_system, (F_e))
print("Equilibrium solved for F_e: ", equilibrium_points)
display(Math(vlatex(equilibrium_points)))
#%%


# finding the jacobian with respect to states (A) and inputs (B)
A = state_variable_form.jacobian(states)
B = state_variable_form.jacobian(inputs)

# sub in values for equilibrium points (x_e, u_e) or (x_0, u_0)
A_lin = simplify(A.subs([(zd,0.), (z,0.), (F, 0.)]))
B_lin = simplify(B.subs([(zd,0.), (z,0.), (F, 0.)]))

display(Math(vlatex(A_lin)))
display(Math(vlatex(B_lin)))


# %%