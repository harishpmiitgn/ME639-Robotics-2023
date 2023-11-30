import numpy as np

def end_effector_position(theta1, theta2, d3):
    # Define link lengths
    L1 = 1
    L2 = 1

    # Define transformation matrices
    T1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                   [np.sin(theta1), np.cos(theta1), 0, 0],
                   [0, 0, 1, L1],
                   [0, 0, 0, 1]])
    T2 = np.array([[np.cos(theta2), 0, np.sin(theta2), 0],
                   [0, 1, 0, 0],
                   [-np.sin(theta2), 0, np.cos(theta2), 0],
                   [0, 0, 0, 1]])
    T3 = np.array([[1, 0, 0, L2],
                   [0, 1, 0, d3],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    # Calculate end effector position
    T = T1 @ T2 @ T3
    p = T[:3,-1]

    return p

# Example usage
theta1 = np.pi/4
theta2 = np.pi/4
d3 = 0.5
p = end_effector_position(theta1, theta2, d3)
print(p)
