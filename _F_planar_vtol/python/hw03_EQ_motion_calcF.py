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
t, m_l, m_r, m_c, d, g, J_c = symbols('t, m_l, m_r, m_c, d, g, J_c')

x_v = dynamicsymbols('x_v')
h = dynamicsymbols('h')
theta = dynamicsymbols('theta')


#defining generalized coords and derivatives
q = Matrix([[h], [x_v], [theta]])
qdot = q.diff(t)

#defining the kinetic energy
p_mc = Matrix([[x_v], [h], [0]])
p_mr = Matrix([[x_v + d*cos(theta)], [h+sin(theta)*d], [0]])
p_ml = Matrix([[x_v - d*cos(theta)], [h-sin(theta)*d], [0]])

v_mc = diff(p_mc, t)
v_mr = diff(p_mr, t)
v_ml = diff(p_ml, t)


omega = Matrix([[0], [0], [diff(theta,t)]])
J = Matrix([[0, 0, 0], [0, 0, 0], [0, 0, J_c]])

#%%
m_l = m_r
K = simplify(0.5*m_l*v_ml.T*v_ml + 0.5*m_r*v_mr.T*v_mr + 0.5*omega.T*J*omega + 0.5*m_c*v_mc.T*v_mc)

display(Math(vlatex(K)))
# just grabbing the scalar inside this matrix so that we can do L = K-P, since P is a scalar
K = K[0,0]

#%%
#defining potential energy 
P = m_c*g*h + m_r*g*(h+sin(theta)*d) + m_l*g*(h-sin(theta)*d)

#calculate the lagrangian, using simplify intermittently can help the equations to be
#simpler, there are also options for factoring and grouping if you look at the sympy
#documentation.
L = simplify(K-P)
print("L =")
display(Math(vlatex(L)))

#%%
# Solution for Euler-Lagrange equations, but this does not include right-hand side (like friction and tau)
Left_side_EQ_Motion_E = simplify( diff(diff(L, qdot), t) - diff(L, q) )

display(Math(vlatex(Left_side_EQ_Motion_E)))

#%%
## Define Generalized Forces
# defining symbols for external force and friction
F, tau = 
r1 = Matrix([[ell*cos(theta)], [ell*sin(theta)], [0]])
F1 = Matrix([[0], [F], [0]])
#Dot product of the derivative of r1 with respect to z and the force F1
Q1 = diff(r1,z).dot(F1)

Q2 = diff(r1,theta).dot(F1)
Q = Matrix([[Q1], [Q2]])
display(Math(vlatex(Q)))
#%% 
############################################################
### Including friction and generalized forces, then solving for highest order derivatives
############################################################

# these are just convenience variables
zd = z.diff(t)
zdd = zd.diff(t)
thetad = theta.diff(t)
thetadd = thetad.diff(t)


# defining the right-hand side of the equation and combining it with E-L part
RHS = Q
full_eom = Left_side_EQ_Motion_E - RHS

#%%
# finding and assigning zdd and thetadd
result = simplify(sp.solve(full_eom, (zdd, thetadd)))

# result is a Python dictionary, we get to the entries we are interested in
# by using the name of the variable that we were solving for
zdd_eom = result[zdd]  # EOM for zdd, as a function of states and inputs
thetadd_eom = result[thetadd] # EOM for thetadd, as a function of states and inputs

display(Math(vlatex(zdd_eom)))
display(Math(vlatex(thetadd_eom)))

mass_matrix = Matrix([[m_l+m_r, m_l*ell/2*cos(theta)], [m_l*ell/2*cos(theta), m_l*ell**2/3]])

#%% [markdown]
# Let's take a look at the inverse of the mass matrix (to compare against the solution shown in the book). 

# %%
display(Math(vlatex(mass_matrix.inv())))



#%% [markdown]
# OK, now we can see if I can get the state variable form of the equations of motion.

#%%
# defining fixed parameters that are not states or inputs (like g, ell, m_l, m_r, b)
params = [(m_l, 0.25), (m_r, 1.), (ell, 1.), (g, 9.8), (b, 0.05)] #this a python list of tuples.

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
