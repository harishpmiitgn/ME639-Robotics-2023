#!/usr/bin/env python
# coding: utf-8

# TASK 3

# In[2]:


import numpy as np

def manipulator_jacobian_DH(num_links, dh_params, joint_types=None):
    # Default joint types: 'R' for revolute joints
    if joint_types is None:
        joint_types = ['R'] * num_links

    # Initialize Jacobian matrix
    jacobian = np.zeros((6, num_links))

    # Iterate through each link to compute Jacobian columns
    for i in range(num_links):
        theta_i = dh_params[i][0] if joint_types[i] == 'R' else 0  # Revolute joint or prismatic joint
        a_i = dh_params[i][1]
        d_i = dh_params[i][2]
        alpha_i = dh_params[i][3]

        # Compute Jacobian columns
        jacobian[:, i] = np.array([
            -a_i * np.sin(theta_i),
            a_i * np.cos(theta_i) * np.cos(alpha_i),
            a_i * np.cos(theta_i) * np.sin(alpha_i),
            0 if joint_types[i] == 'R' else 1,  # Revolute joint or prismatic joint
            0 if joint_types[i] == 'R' else 0,
            0
        ])

    # End-effector position
    end_effector_position = np.array([
        dh_params[num_links - 1][1],  # a_n
        0,  # b_n
        dh_params[num_links - 1][2]   # c_n
    ])

    # End-effector velocity
    end_effector_velocity = np.dot(jacobian, np.ones(num_links))

    return jacobian, end_effector_position, end_effector_velocity

# Example usage
num_links = 3
# Example DH parameters: [theta, a, d, alpha]
dh_params = np.array([
    [0, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 0, 0]
])
# Example joint types: 'R' for revolute, 'P' for prismatic
joint_types = ['R', 'R', 'R']

jacobian, end_effector_position, end_effector_velocity = manipulator_jacobian_DH(num_links, dh_params, joint_types)

print("Complete Manipulator Jacobian:")
print(jacobian)

print("\nEnd-Effector Position:")
print(end_effector_position)

print("\nEnd-Effector Velocity:")
print(end_effector_velocity)


# TASK 4 STANFORD

# In[4]:


# RRP Configuration for Stanford Manipulator
num_links_stanford_rrp = 3

# Define DH parameters: [theta, a, d, alpha]
dh_params_stanford_rrp = np.array([
    [0, 0, 0, np.pi/2],
    [0, 0.2, 0, 0],
    [0, 0.2, 0, 0]
])

# Joint types: 'R' for revolute, 'P' for prismatic
joint_types_stanford_rrp = ['R', 'R', 'P']

# Compute Jacobian, end-effector position, and end-effector velocity for Stanford RRP
jacobian_stanford_rrp, end_effector_position_stanford_rrp, end_effector_velocity_stanford_rrp = manipulator_jacobian_DH(
    num_links_stanford_rrp, dh_params_stanford_rrp, joint_types_stanford_rrp)

print("Stanford RRP Manipulator:")
print("Complete Manipulator Jacobian:")
print(jacobian_stanford_rrp)

print("\nEnd-Effector Position:")
print(end_effector_position_stanford_rrp)

print("\nEnd-Effector Velocity:")
print(end_effector_velocity_stanford_rrp)


# TASK 4 SCARA 

# In[5]:


# SCARA Configuration for SCARA Manipulator
num_links_scara = 3

# Define DH parameters for SCARA: [theta, a, d, alpha]
dh_params_scara = np.array([
    [0, 0.2, 0, 0],
    [0, 0.2, 0, 0],
    [0, 0, 0, 0]
])

# Joint types: 'R' for revolute, 'R' for revolute, 'P' for prismatic
joint_types_scara = ['R', 'R', 'P']

# Compute Jacobian, end-effector position, and end-effector velocity for SCARA
jacobian_scara, end_effector_position_scara, end_effector_velocity_scara = manipulator_jacobian_DH(
    num_links_scara, dh_params_scara, joint_types_scara)

print("\nSCARA Manipulator:")
print("Complete Manipulator Jacobian:")
print(jacobian_scara)

print("\nEnd-Effector Position:")
print(end_effector_position_scara)

print("\nEnd-Effector Velocity:")
print(end_effector_velocity_scara)


# In[ ]:




