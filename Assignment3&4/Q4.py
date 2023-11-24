import Q3
import numpy as np

dh_params_stanford = [{'theta': np.pi/4, 'd': 1, 'a': 0, 'alpha': np.pi/2}, 
                      {'theta': np.pi/4, 'd': 0, 'a': 1, 'alpha': 0}, 
                      {'theta': 0, 'd': 1, 'a': 0, 'alpha': 0}]
joint_vars_stanford = [np.pi/4, 1, 1]
joint_vels_stanford = [1, 1, 1]
joint_types_stanford = ['R', 'R', 'P']

J_stanford, ee_pos_stanford, ee_vel_stanford = Q3.compute_kinematics(3, dh_params_stanford, joint_vars_stanford, joint_vels_stanford, joint_types_stanford)

print("Stanford Manipulator:")
print("Jacobian:")
print(J_stanford)
print("End-effector position:")
print(ee_pos_stanford)
print("End-effector velocity:")
print(ee_vel_stanford)


dh_params_scara = [{'theta': np.pi/4, 'd': 0, 'a': 1, 'alpha': 0}, 
                   {'theta': np.pi/4, 'd': 0, 'a': 1, 'alpha': 0}, 
                   {'theta': 0, 'd': 1, 'a': 0, 'alpha': 0}]
joint_vars_scara = [np.pi/4, np.pi/4, 1]
joint_vels_scara = [1, 1, 1]
joint_types_scara = ['R', 'R', 'P']

J_scara, ee_pos_scara, ee_vel_scara = Q3.compute_kinematics(3, dh_params_scara, joint_vars_scara, joint_vels_scara, joint_types_scara)

print("SCARA Manipulator:")
print("Jacobian:")
print(J_scara)
print("End-effector position:")
print(ee_pos_scara)
print("End-effector velocity:")
print(ee_vel_scara)
