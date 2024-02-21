#%%
# This is the code for the linearization case study in the E section of the homework.
from sympy import sin, cos, Matrix, symbols, Function, simplify, latex, eye, monic
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vpprint, vlatex
from IPython.display import Math, display


#defining mathematical variables (called symbols in sympy) and time varying functions like z and theta and h
t, m1, m2, ell, g, F, ze = symbols('t, m1, m2, ell, g, F, z_e')
z = dynamicsymbols('z')
theta = dynamicsymbols('theta')


# this should be the equations of motion in state variable form (except for equations that relate derivatives of state
# to existing states (e.g. d(theta)/dt = theta_dot). We could get this from Euler-Lagrange equations, but we are just
# going to use the equations of motion from the previous solution. 
f_of_x_and_u = Matrix([[z.diff(t)],
        [theta.diff(t)], 
        [1/m1*(-m1*g*sin(theta)+m1*z*theta.diff(t)**2)], 
        [1/(m2*ell**2/3+m1*z**2)*(ell*F*cos(theta)-2*m1*z*z.diff(t)*theta.diff(t)-m1*g*z*cos(theta)-m2*g*ell/2*cos(theta))]])

print("EOM, solved for second derivatives of generalized coordinates :\n")
display(Math(vlatex((f_of_x_and_u))))

# defining states and inputs symbolically
states = (Matrix([z, theta, z.diff(t), theta.diff(t)])).reshape(4,1)
inputs = Matrix([F])

# finding the jacobian with respect to states (A) and inputs (B)
A = f_of_x_and_u.jacobian(states)
B = f_of_x_and_u.jacobian(inputs)

# substituting in the equilibrium values for each state and input (finding the equilibrium points can likely be
# done automatically in sympy as well, but we are currently defining them by hand)
A_lin = simplify(A.subs([(theta.diff(t),0), (theta, 0), (z.diff(t), 0), (F, m1*g/ell*ze + m2*g/2), (z, ze)]))
B_lin = simplify(B.subs([(theta.diff(t),0), (theta, 0), (z.diff(t), 0), (F, m1*g/ell*ze + m2*g/2), (z, ze)]))

print("Linearized A Matrix is:")
display(Math(vlatex(A_lin)))

print("Linearized B Matrix is:")
display(Math(vlatex(B_lin)))


# we can also transform this to a transfer function if we define C and D matrices
C = Matrix([[0, 1, 0, 0], [0, 0, 0, 1.0]])
D = Matrix([[0], [0]])

# these are the two transfer functions for z and theta with respect to input F
s = symbols('s')
# this is the C(sI-A)^(-1)B + D  equation shown in class to find the transfer function
transfer_func = simplify(C@(s*eye(4)-A_lin).inv()@B_lin+D)   
print("Tranfer functions (without simplifying assumption):")
display(Math(vlatex(transfer_func)))

# now setting the m1*g term equal to zero as described in HW E.5
A_E5 = A_lin.subs([(m1*g, 0)])
B_E5 = B_lin.subs([(m1*g, 0)])

transfer_func_partC = simplify(C*(s*eye(4)-A_E5).inv()*B_E5+D)
print("Tranfer functions (with simplifying assumption):")
display(Math(vlatex(transfer_func_partC)))


# %%