import numpy as np
from math import *

l1 = float(input("l1 = "))     # link length 1
l2 = float(input("l2 = "))     # link length 2
l3 = float(input("l3 = "))     # link length 3
theta1 = int(input("theta1 = "))      # Joint angle 1
theta1 = radians(theta1)
theta2 = int(input("theta2 = "))      # Joint angle 2
theta2 = radians(theta2)
theta3 = int(input("theta3 = "))      # Joint angle 3
theta3 = radians(theta3)

# theta1 = np.radians(45)  # Joint angle 1
# theta2 = np.radians(30)  # Joint angle 2
# theta3 = np.radians(60)  # Joint angle 3
# l1 = 10.0                # Link length 1
# l2 = 10.0                # Link length 2
# l3 = 10.0                # Link length 3

def find_jacobian_matrix(theta1, theta2, theta3, l1, l2, l3):
    # Transformation matrices
    H01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, -l1*np.sin(theta1)],
                   [np.sin(theta1), np.cos(theta1), 0, l1*np.cos(theta1)],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    H12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, -l2*np.sin(theta2)],
                    [np.sin(theta2),  np.cos(theta2), 0, l2*np.cos(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    H23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, -l3*np.sin(theta3)],
                    [np.sin(theta3),  np.cos(theta3), 0, l3*np.cos(theta3)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Final transformation matrix
    H02 = np.dot(H01, H12)
    H03 = np.dot(H02, H23)


    O0 = np.array([[0],
                   [0],
                   [0]])

    O1 = H01[:3, 3:]
    O2 = H02[:3, 3:]
    O3 = H03[:3, 3:]


    Z0 = Z1 = Z2 = np.array([[0],
                             [0],
                             [1]])
        

    J1 = np.zeros((6, 1))
    J1_v = np.cross(Z0.flatten(), (O3 - O0).flatten())
    J1_w = Z0
    J1[:3, 0] = J1_v
    J1[3:, 0] = J1_w.flatten()

    J2 = np.zeros((6, 1))
    J2_v = np.cross(Z1.flatten(), (O3 - O1).flatten())
    J2_w = Z1
    J2[:3, 0] = J2_v
    J2[3:, 0] = J2_w.flatten()

    J3 = np.zeros((6, 1))
    J3_v = np.cross(Z2.flatten(), (O3 - O2).flatten())
    J3_w = Z2
    J3[:3, 0] = J3_v
    J3[3:, 0] = J3_w.flatten()
    J = np.hstack((J1, J2, J3))
    return J

# Calculate Jacobian matrix
jacobian_matrix = find_jacobian_matrix(theta1, theta2, theta3, l1, l2, l3)
print("Jacobian matrix", jacobian_matrix)
