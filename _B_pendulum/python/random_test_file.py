#%% [markdown]
# # Random Test File
import numpy as np
import pendulumParam as P

A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, -3 * P.m1 * P.g / 4 / (.25 * P.m1 + P.m2),
                -P.b / (.25 * P.m1 + P.m2), 0.0],
            [0.0, 
                3*(P.m1+P.m2)*P.g/2/(0.25*P.m1 + P.m2)/P.ell,
                3 * P.b / 2 / (.25 * P.m1 + P.m2) / P.ell, 0.0]])
B = np.array([[0.0],
                           [0.0],
                           [1 / (.25 * P.m1 + P.m2)],
                           [-3.0 / 2 / (.25 * P.m1 + P.m2) / P.ell]])
C = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])
C = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])
Cr = np.array([[1, 0]]) @ C
print("Cr: ", Cr)
# %%
Cr = np.array([[1, 0]]) @ C # selects the first row of 
A1 = np.concatenate((
                np.concatenate((A, np.zeros((4, 1))), axis=1),
                np.concatenate((-Cr, np.matrix([[0.0]])), axis=1)),
                axis=0)

B1 = np.concatenate((B, np.matrix([[0.0]])), axis=0)

# %%
