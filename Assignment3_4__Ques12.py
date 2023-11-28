import numpy as np



# End effector position: Assign specific values to a, b, and c
a = 1
b = 2
c = 1

d_joint  = 1.0  
def calculate_inverse_kinematics(a, b, c) -> np.ndarray:
    theta_joint1 = np.arctan2(b, a)

    distance_a = np.sqrt(a**2 + b**2)
    distance_b = c - d_joint 

    theta_joint2 = np.arctan2(distance_b, distance_a) + np.pi/2
    d33 = np.sqrt(distance_a**2 + distance_b**2)

    return np.array([theta_joint1, theta_joint2, d33])

joint_state = calculate_inverse_kinematics(a, b, c)
print("Inverse Kinematics Result:", joint_state)
