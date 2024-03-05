#%% [markdown]
## Problem E.8
from transfer_functions_case_study_E import *
# %% [markdown]
# Part b
zeta_theta = 0.707
omega_n = 2.2
import blockbeamParam as P
b0 = (P.length)/(P.m2*P.length**2/3 + P.m1*(P.length/2)**2)
k_d_theta = 2*zeta_theta*omega_n/b0
print('k_d_theta: ', k_d_theta)
k_p_theta = omega_n**2/b0
print('k_p_theta: ', k_p_theta)
d# %% [markdown]
