import math
import numpy as np

def scara_ik(x, y, z, d, l1, l2):
    # Calculate the distance from the origin to the end effector position
    r = math.sqrt(x**2 + y**2)

    # Calculate θ2 using the law of cosines
    cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.acos(cos_theta2)

    # Calculate θ1 using trigonometry
    sin_theta2 = math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2((l2 * sin_theta2), (l1 + l2 * cos_theta2))

    # Calculate theta3 (z3) using the vertical position
    z3 = z - d

    return theta1,theta2,z3

def scara_forward_kinematics(theta1, theta2, z, d, l1, l2):

    # Forward kinematics calculations
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    z = z + d  # Adjust the output z by adding the height of the first link
    return x, y, z

# Example usage:
x = 4.0  # X position of the end-effector
y = 3.0  # Y position of the end-effector
z = 2.0  # Z position of the end-effector
d = 1.0  # Height of the first link above the XY plane
l1 = 5.0  # Length of the first and second links
l2 = 5.0  # Length of the third link
print("sample values: \nend effector positions: x,y,z \nlink lengths 5.0,5.0 \noffest: 1.0\n")
theta1, theta2, z3 = scara_ik(x, y, z, d, l1, l2)
print(f"Theta1: {theta1} radians")
print(f"Theta2: {theta2} radians")
print(f"Z3: {z3} units above the XY plane")

x, y, z = scara_forward_kinematics(theta1, theta2, z3, d, l1, l2)
print(f'End-effector position (x, y, z): ({x}, {y}, {z})')
