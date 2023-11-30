#!/usr/bin/env python
# coding: utf-8

# Q1

# In[2]:


import numpy as np 
import matplotlib.pyplot as plt

x=float(input("end effector x coordinate: "))
y=float(input("end effector y coordinate: "))
z=float(input("end effector z coordinate: "))

l1=float(input("Link length 1: "))
l2=float(input("Link length 2: "))
l3=float(input("Link length 3: "))

def getH(R, d):
    H=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H

def getR(q):
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

#now we define q1, d and q2

q1 = np.arctan2(x,y)

d = np.sqrt(x**2+y**2+(z-l1)**2)-(l2+l3)

q2 = np.arctan2(z-l1, np.sqrt(x**2+y**2))

#Now for getting H and R:

H01=getH(getR(q1), [[0],[0],[0]])
H12=getH(np.matmul([[1, 0, 0],[0, 0, -1],[0, 1, 0]], getR(q2)), [[0],[0],[l1]])
H23=getH(getR(0), [[l2+l3],[0],[0]])
P=np.linalg.multi_dot([H01,H12,H23,[[d],[0],[0],[1]]])

#printing answers rounded off to 2 decimal places wrt the base frame

print(f"\n position of the end effector from inverse solution: {P[0][0].round(decimals=2)}i {P[1][0].round(decimals=2)}j {P[2][0].round(decimals=2)}k with respect to the base frame\n") 

# Verified it for (x,y,z) as (1,1,1) and l1,l2,l3 as (0.1,0.1,0.1)


# Q5

# In[4]:


import numpy as np
from math import atan2, sqrt

"""
This can also be done by inputing the rotation matrix by using the function getR

def getR(q):
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R
    
R01 = getR(q1)
R12 = np.matmul([[1, 0, 0],[0, 0, -1],[0, 1, 0]], getR(q2))
R23 = getR(0)   

"""

def inverse_spherical_wrist(r):
    # Given rotation matrix R30
    r13, r23, r33, r32 = r[0, 2], r[1, 2], r[2, 2], r[2,1]
    r11, r21, r12, r22 = r[0, 0], r[1, 0], r[0, 1], r[1, 1]

    # Solve for theta5
    c1 = np.arctan2(r23, r33)
    s1 = np.arctan2(-r13, np.sqrt(r23**2 + r33**2))

    c5 = np.cos(s1)
    if abs(c5) > 1e-6:
        s5 = r33 / c5
        theta5 = np.arctan2(s1 * r13 - c1 * r23, c1 * r13 + s1 * r23 - c5)
        
        # Solve for theta4
        theta4 = np.arctan2(c1 * r23 - s1 * r13, -c1 * r33 + s1 * r23)
        
        # Solve for theta6
        theta6 = np.arctan2(-s1 * r12 + c1 * r22, s1 * r11 - c1 * r21)
        
        return theta4, theta5, theta6
    else:
        # Singular configuration: z3 and z5 are collinear
        theta4 = np.arctan2(r21, r11)
        theta6 = np.arctan2(r32, -r31)
        
        return theta4, theta6

# Example rotation matrix R30
R30 = np.array([
    [0.866, -0.5, 0],
    [0.5, 0.866, 0],
    [0, 0, 1]
])

# Solve for inverse kinematics
theta4, theta5, theta6 = inverse_spherical_wrist(R30)
print(f"Theta 4: {theta4}")
print(f"Theta 5: {theta5}")
print(f"Theta 6: {theta6}")


# Q6

# In[5]:


import numpy as np
import sympy as sp

#I have tried to use the sympy library which a symbolic python library used to solve differentials
# and is easily integrable with numpy. Please do let me know if I did anything wrong! 

# Constants
m1 = int(input("Enter m1: "))
m2 = int(input("Enter m2: "))
m3 = int(input("Enter m3: "))
l1 = int(input("Enter l1: "))
l2 = int(input("Enter l2: "))
l3 = int(input("Enter l3: "))

g = 9.81  # Acceleration due to gravity

# Define the symbolic variables
q1, q2, q3 = sp.symbols('q1 q2 q3')
q1_dot, q2_dot, q3_dot = sp.symbols('q1_dot q2_dot q3_dot')
tau1, tau2, tau3 = sp.symbols('tau1 tau2 tau3')

# Define symbolic expressions for cosine and sine terms
c1, s1 = sp.cos(q1), sp.sin(q1)
c12, s12 = sp.cos(q1 + q2), sp.sin(q1 + q2)
c123, s123 = sp.cos(q1 + q2 + q3), sp.sin(q1 + q2 + q3)

# Kinematics (position vectors)
x1 = l1 * c1
y1 = l1 * s1
x2 = x1 + l2 * c12
y2 = y1 + l2 * s12
x3 = x2 + l3 * c123
y3 = y2 + l3 * s123

# Kinetic and potential energies
T = 0.5 * m1 * (sp.diff(x1, q1) ** 2 + sp.diff(y1, q1) ** 2) * q1_dot ** 2 + \
    0.5 * m2 * (sp.diff(x2, q1) ** 2 + sp.diff(y2, q1) ** 2) * (q1_dot + q2_dot) ** 2 + \
    0.5 * m3 * (sp.diff(x3, q1) ** 2 + sp.diff(y3, q1) ** 2) * (q1_dot + q2_dot + q3_dot) ** 2

V = m1 * g * y1 + m2 * g * y2 + m3 * g * y3

# Lagrangian
L = T - V

# Compute the equations of motion using Lagrange's equations
eq1 = sp.diff(sp.diff(L, q1_dot), 't') - sp.diff(L, q1) - tau1
eq2 = sp.diff(sp.diff(L, q2_dot), 't') - sp.diff(L, q2) - tau2
eq3 = sp.diff(sp.diff(L, q3_dot), 't') - sp.diff(L, q3) - tau3

# Define the system of differential equations
state_symbols = [q1, q2, q3, q1_dot, q2_dot, q3_dot]
forcing_functions = [tau1, tau2, tau3]

# Convert symbolic expressions to numpy-ready functions
eq1_np = sp.lambdify(state_symbols + forcing_functions, eq1)
eq2_np = sp.lambdify(state_symbols + forcing_functions, eq2)
eq3_np = sp.lambdify(state_symbols + forcing_functions, eq3)

# Simulation parameters
time_step = 0.01
total_time = 10.0
num_steps = int(total_time / time_step)

# Initial conditions and small torques
initial_conditions = [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]  # Small initial angles and velocities
small_torques = [0.001, 0.001, 0.001]  # Small torques applied as told to ensure that from continuosly accelerating.

# Simulation loop
state = np.array(initial_conditions)
for i in range(num_steps):
    torque_values = np.array(small_torques)
    new_state = state + np.array([
        eq1_np(*state, *torque_values),
        eq2_np(*state, *torque_values),
        eq3_np(*state, *torque_values),
        state[3], state[4], state[5]
    ]) * time_step
    state = new_state

# The results can be plotted to show the output using the matplotlib libraray if required, didn't do it 
# since it was not asked in the question.


# In[6]:


def desired_trajectory(t):
    q1_desired = np.sin(t)
    q2_desired = np.cos(2 * t)
    q3_desired = np.sin(3 * t)
    return [q1_desired, q2_desired, q3_desired]

def stochastic_disturbance():
    return np.random.normal(0, 0.01, 3)  

# Simple Independent Joint Control
def simple_control(q, q_dot, q_desired):
    return [kp * (q_desired[i] - q[i]) - kd * q_dot[i] for i in range(len(q))]

def diff_eq(t, state, tau_values):
    q1, q2, q3, q1_dot, q2_dot, q3_dot = state
    tau1, tau2, tau3 = tau_values
    
    # Get the desired trajectory values at time t
    q_desired = desired_trajectory(t)
    q_dot_desired = [1, 1, 1]  # Random desired velocities
    q_ddot_desired = [0.1, 0.1, 0.1]  # Random desired accelerations
    
    # Implement the desired control strategy here
    control_torques = simple_control([q1, q2, q3], [q1_dot, q2_dot, q3_dot], q_desired)
    
    # Add small stochastic disturbance to the torques
    disturbance = stochastic_disturbance()
    control_torques = [control_torques[i] + disturbance[i] for i in range(len(control_torques))]
    
    dq_dt = dynamics_func(q1, q2, q3, q1_dot, q2_dot, q3_dot, control_torques[0], control_torques[1], control_torques[2])
    return [q1_dot, q2_dot, q3_dot, dq_dt[0], dq_dt[1], dq_dt[2]]


# 

# In[ ]:




