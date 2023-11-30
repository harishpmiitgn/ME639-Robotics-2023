import math
import numpy as np

def calculate_scara_forward_kinematics(theta1, theta2, z3, first_link_height, link1_length, link2_length):
    # Forward kinematics calculations
    end_effector_x = link1_length * np.cos(theta1) + link2_length * np.cos(theta1 + theta2)
    end_effector_y = link1_length * np.sin(theta1) + link2_length * np.sin(theta1 + theta2)
    end_effector_z = z3 + first_link_height  # Adjust the output z by adding the height of the first link

    return end_effector_x, end_effector_y, end_effector_z

def calculate_scara_inverse_kinematics(end_effector_x, end_effector_y, end_effector_z, first_link_height, link1_length, link2_length):
    # Calculate the distance from the origin to the end effector position
    radial_distance = math.sqrt(end_effector_x**2 + end_effector_y**2)

    # Calculate θ2 using the law of cosines
    cos_theta2 = (radial_distance**2 - link1_length**2 - link2_length**2) / (2 * link1_length * link2_length)
    theta2 = math.acos(cos_theta2)

    # Calculate θ1 using trigonometry
    sin_theta2 = math.sin(theta2)
    theta1 = math.atan2(end_effector_y, end_effector_x) - math.atan2((link2_length * sin_theta2), (link1_length + link2_length * cos_theta2))

    # Calculate z3 using the vertical position
    z3 = end_effector_z - first_link_height

    return theta1, theta2, z3

# Example usage:
end_effector_x = 2.0
end_effector_y = 2.0
end_effector_z = 2.0
first_link_height = 1.5
link1_length = 3.0
link2_length = 3.0

print("Sample values:")
print(f"End effector positions: ({end_effector_x}, {end_effector_y}, {end_effector_z})")
print(f"Link lengths: {link1_length}, {link2_length}")
print(f"Offset: {first_link_height}\n")

theta1, theta2, z3 = calculate_scara_inverse_kinematics(end_effector_x, end_effector_y, end_effector_z, first_link_height, link1_length, link2_length)

print("Forward Kinematics Results:")
print(f"Calculated end-effector position: ({calculated_end_effector_x}, {calculated_end_effector_y}, {calculated_end_effector_z})")

print("\nInverse Kinematics Results:")
print(f"Theta1: {theta1} radians")
print(f"Theta1: {np.degrees(theta1):.2f} degrees")

print(f"Theta2: {theta2} radians")
print(f"Theta2: {np.degrees(theta2):.2f} degrees")

print(f"Z3: {z3} units above the XY plane")

calculated_end_effector_x, calculated_end_effector_y, calculated_end_effector_z = calculate_scara_forward_kinematics(theta1, theta2, z3, first_link_height, link1_length, link2_length)
