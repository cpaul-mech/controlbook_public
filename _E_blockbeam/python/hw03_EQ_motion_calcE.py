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
t, m1, m2, ell, g, Jz = symbols('t, m1, m2, ell, g, Jz')

z = dynamicsymbols('z')
theta = dynamicsymbols('theta')


#defining generalized coords and derivatives
q = Matrix([[z], [theta]])
qdot = q.diff(t)

#defining the kinetic energy
p1 = Matrix([[z*cos(theta)], [z*sin(theta)], [0]])
p2 = Matrix([[((ell*cos(theta))/2)], [(ell*sin(theta))/2], [0]])

v1 = diff(p1, t)
v2 = diff(p2, t)
step1 = v1.T*v1
step2 = v2.T*v2
# display(Math(vlatex(step1)))
# display(Math(vlatex(step2)))
Jz = (m2*ell**2)/12.0
omega = Matrix([[0], [0], [diff(theta,t)]])
J = Matrix([[0, 0, 0], [0, 0, 0], [0, 0, Jz]])

#%%
K = (0.5*m1*step1 + 0.5*m2*step2 + 0.5*omega.T*J*omega)
display(Math(vlatex(K)))
K = simplify(K)
display(Math(vlatex(K)))
# just grabbing the scalar inside this matrix so that we can do L = K-P, since P is a scalar
K = K[0,0]


#%%
#defining potential energy 
P = m1*g*z*sin(theta) + m2*g*(z*sin(theta))

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
F, b = symbols('F, b')
r1=Matrix([[ell*cos(theta)], [ell*sin(theta)], [0]])
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

# display(Math(vlatex(zdd_eom)))
# display(Math(vlatex(thetadd_eom)))
display(Math(vlatex(result)))

#%% [markdown] 
# Although not covered in this class, there are ways to store this function and just re-load it
# when you want to use it, so that you don't have to re-run the sympy code every time you want to
# perform a simulation. See https://stackoverflow.com/questions/29079923/save-load-sympy-lambdifed-expressions


# %%
