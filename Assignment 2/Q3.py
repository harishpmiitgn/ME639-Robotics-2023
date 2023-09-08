import numpy as np
from math import radians, cos, sin

l1 = float(input("l1 = "))     # link length 1
l2 = float(input("l2 = "))     # link length 2
l3 = float(input("l3 = "))     # link length 3
theta1 = int(input("theta1 = "))      # Joint angle 1
theta1 = radians(theta1)
theta2 = int(input("theta2 = "))      # Joint angle 2
theta2 = radians(theta2)

# theta1 = np.radians(45)  # Joint angle 1
# theta2 = np.radians(30)  # Joint angle 2
# l1 = 10.0                # Link length 1
# l2 = 10.0                # Link length 2
# l3 = 10.0                # Link length 3

def scara_position(theta1, theta2, l1, l2, l3):
    # Transformation matrices
    H1 = np.array([[cos(theta1), -sin(theta1), 0, 0],
                   [sin(theta1), cos(theta1), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    H2 = np.array([[cos(theta2), -sin(theta2), 0, l1],
                   [sin(theta2), cos(theta2), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    H3 = np.array([[1, 0, 0, l2],
                   [0, 1, 0, 0],
                   [0, 0, 1, -l3],
                   [0, 0, 0, 1]])

    # Final transformation matrix
    H = np.dot(np.dot(H1, H2), H3)

    # End effector position vector
    return H[:3, 3]

# Calculate end effector position
end_effector_position = scara_position(theta1, theta2, l1, l2, l3)
print("End Effector Position is ", end_effector_position)
