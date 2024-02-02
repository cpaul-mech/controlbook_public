#%%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display


#%%
####################################################################################
#example from Case Study B (with vectors/matrices)
####################################################################################
# importing these functions directly to make life a little easier and code a little more readable
from sympy import sin, cos, diff, Matrix, symbols, Function, pretty_print, simplify, init_printing, latex
#init_printing(use_latex=True)

#defining mathematical variables (called symbols in sp) and time varying functions like z and theta
k, b, F, m1, t = symbols('k, b, F, m1, t')

z = dynamicsymbols('z')
z_dot = z.diff()



#defining generalized coords and derivatives
q = Matrix([[z]])
qdot = q.diff(t)

#defining the kinetic energy

K = 0.5*m1*z_dot**2

#%%
#defining potential energy 
P = 0.5*k*z**2

#calculate the lagrangian, using simplify intermittently can help the equations to be
#simpler, there are also options for factoring and grouping if you look at the sympy
#documentation.
L = simplify(K-P)

#%%
# Solution for Euler-Lagrange equations, but this does not include right-hand side (like friction and tau)
EL_case_studyB = simplify( diff(diff(L, qdot), t) - diff(L, q) )

display(Math(vlatex(EL_case_studyB)))



#%% 
############################################################
### Including friction and generalized forces, then solving for highest order derivatives
############################################################

# these are just convenience variables
zd = z.diff(t)
zdd = zd.diff(t)

# defining symbols for external force and friction
F, b = symbols('F, b')

# defining the right-hand side of the equation and combining it with E-L part
RHS = Matrix([[F - b*zd]])
full_eom = EL_case_studyB - RHS
display(Math(vlatex(full_eom)))
#%%
# finding and assigning zdd and thetadd
result = simplify(sp.solve(full_eom, (zdd)))

# result is a Python dictionary, we get to the entries we are interested in
# by using the name of the variable that we were solving for
zdd_eom = result[zdd]  # EOM for zdd, as a function of states and inputs


display(Math(vlatex(result)))



#%% [markdown]
# Let's take a look at the inverse of the mass matrix (to compare against the solution shown in the book). 

# %%
display(Math(vlatex(mass_matrix.inv())))



#%% [markdown]
# OK, now we can see if I can get the state variable form of the equations of motion.

#%%
# defining fixed parameters that are not states or inputs (like g, ell, m1, m2, b)
params = [(m1, 0.25), (m2, 1.), (ell, 1.), (g, 9.8), (b, 0.05)] #this a python list of tuples.

# substituting parameters into the equations of motion
zdd_eom = zdd_eom.subs(params)
thetadd_eom = thetadd_eom.subs(params)

# now defining the state variables that will be passed into f(x,u) 
state = [zd, z, thetad, theta]
ctrl_input = [F]

# defining the function that will be called to get the derivatives of the states
state_dot = [zdd_eom, zd, thetadd_eom, thetad]

# converting the function to a callable function that uses numpy to evaluate and 
# return a list of state derivatives
f = sp.lambdify(state + ctrl_input, state_dot)

# calling the function as a test to see if it works, could do something like this 
# in a simulation

# f(zd, z, thetad, theta, F)
print("x_dot = ", f(0, 0, 0, 0, 1))

#%% [markdown] 
# Although not covered in this class, there are ways to store this function and just re-load it
# when you want to use it, so that you don't have to re-run the sympy code every time you want to
# perform a simulation. See https://stackoverflow.com/questions/29079923/save-load-sympy-lambdifed-expressions


# %%
