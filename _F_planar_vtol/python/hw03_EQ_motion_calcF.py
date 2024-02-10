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

z = dynamicsymbols('z')
h = dynamicsymbols('h')
theta = dynamicsymbols('theta')


#defining generalized coords and derivatives
q = Matrix([[z], [h], [theta]])
qdot = q.diff(t)

#defining the kinetic energy
p_mc = Matrix([[z], [h], [0.]])
p_mr = Matrix([[z + d*cos(theta)], [h + d*sin(theta)], [0.]])
p_ml = Matrix([[z - d*cos(theta)], [h - d*sin(theta)], [0.]])

v_c = diff(p_mc, t)
v_r = diff(p_mr, t)
v_l = diff(p_ml, t)


omega = Matrix([[0], [0], [diff(theta,t)]])
J = Matrix([[0, 0, 0], [0, 0, 0], [0, 0, J_c]])

#%%
K = (0.5*m_c*v_c.T*v_c + 0.5*omega.T*J*omega + 0.5*m_r*v_r.T*v_r + 0.5*m_l*v_l.T*v_l)
K = K.subs(m_l, m_r)
K = simplify(K)
display(Math(vlatex(K)))
# just grabbing the scalar inside this matrix so that we can do L = K-P, since P is a scalar
K = K[0,0]

#%%
#defining potential energy 
P = m_c*g*h + m_r*g*(h+sin(theta)*d) + m_l*g*(h-sin(theta)*d)
P = P.subs(m_l, m_r)
P = simplify(P)
display(Math(vlatex(P)))
#%%
#calculate the lagrangian, using simplify intermittently can help the equations to be
#simpler, there are also options for factoring and grouping if you look at the sympy
#documentation.

L = simplify(K-P)


print("L =")
display(Math(vlatex(L)))

#%%
# Solution for Euler-Lagrange equations, but this does not include right-hand side (like friction and tau)
Left_side_EQ_Motion_E = simplify( diff(diff(L, qdot), t) - diff(L, q) )
print("Left Hand Side EOM =")
display(Math(vlatex(Left_side_EQ_Motion_E)))

#%%
## Define Generalized Forces
# defining symbols for external force and friction
F, tau = symbols('F tau')
RHS = Matrix([[-F*sin(theta)],[F*cos(theta)], [tau]])
print("Right Hand Side EOM =")
display(Math(vlatex(RHS)))
#%%
# defining the full equation of motion
full_eom = Left_side_EQ_Motion_E - RHS
print("Full EOM =")
display(Math(vlatex(full_eom)))
#%% 
############################################################
### Including friction and generalized forces, then solving for highest order derivatives
############################################################

# these are just convenience variables
zd = z.diff(t)
zdd = zd.diff(t)
thetad = theta.diff(t)
thetadd = thetad.diff(t)
hd = h.diff(t)
hdd = hd.diff(t)


#%%
# finding and assigning zdd and thetadd
result = simplify(sp.solve(full_eom, (zdd, hdd, thetadd)))

# result is a Python dictionary, we get to the entries we are interested in
# by using the name of the variable that we were solving for
zdd_eom = result[zdd]  # EOM for zdd, as a function of states and inputs
hdd_eom = result[hdd]  # EOM for hdd, as a function of states and inputs
thetadd_eom = result[thetadd] # EOM for thetadd, as a function of states and inputs
print("zdd_eom =")
display(Math(vlatex(zdd_eom)))
print("hdd_eom =")
display(Math(vlatex(hdd_eom)))
print("thetadd_eom =")
display(Math(vlatex(thetadd_eom)))

# %%
