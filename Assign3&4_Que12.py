import numpy as np

def calculate_forward_kinematics(link1_length, link2_length, link3_length, joint_angle_1, joint_angle_2, prismatic_displacement):
    # Calculate the position of the end of link2 (W)
    Wx = link2_length * np.cos(joint_angle_1) * np.cos(joint_angle_2)
    Wy = link2_length * np.sin(joint_angle_1) * np.cos(joint_angle_2)
    Wz = link2_length * np.sin(joint_angle_2)

    # Calculate the position of the end-effector (E)
    end_effector_x = Wx
    end_effector_y = Wy
    end_effector_z = Wz + prismatic_displacement

    return end_effector_x, end_effector_y, end_effector_z

def calculate_inverse_kinematics(link1_length, link2_length, link3_length, end_effector_x, end_effector_y, end_effector_z):
    # Calculate the first joint angle (theta1)
    joint_angle_1 = np.arctan2(end_effector_y, end_effector_x)

    # Calculate the distance from the base to the end-effector projection in the xz plane
    radial_distance = np.sqrt(end_effector_x**2 + end_effector_z**2)

    # Calculate the angle between link2 and the projection of the end-effector in the xz plane
    alpha = np.arctan2(end_effector_z, end_effector_x)

    # Using the Law of Cosines to calculate the angle between link2 and link3 (theta2)
    cos_theta2 = (link2_length**2 + radial_distance**2 - link3_length**2) / (2 * link2_length * radial_distance)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    joint_angle_2 = alpha - np.arctan2(sin_theta2, cos_theta2)

    # Calculate the prismatic displacement (d3) along the common axis of link2 and link3
    prismatic_displacement = end_effector_z - link2_length * np.sin(joint_angle_2)

    return joint_angle_1, joint_angle_2, prismatic_displacement

# Example usage:
link1_length = 1.5
link2_length = 1.5
link3_length = 1.5
end_effector_x = 1.0
end_effector_y = 1.0
end_effector_z = 1.0

print("Sample values:")
print(f"Link lengths: {link1_length}, {link2_length}, {link3_length}")
print(f"Desired coordinates: {end_effector_x}, {end_effector_y}, {end_effector_z}")

joint_angle_1, joint_angle_2, prismatic_displacement = calculate_inverse_kinematics(
    link1_length, link2_length, link3_length, end_effector_x, end_effector_y, end_effector_z
)

print("\nForward Kinematics Results:")
print(f"Calculated end-effector x-coordinate: {calculated_end_effector_x:.2f} units")
print(f"Calculated end-effector y-coordinate: {calculated_end_effector_y:.2f} units")
print(f"Calculated end-effector z-coordinate: {calculated_end_effector_z:.2f} units")

print("\nInverse Kinematics Results:")
print(f"Joint Angle 1: {np.degrees(joint_angle_1):.2f} degrees")
print(f"Joint Angle 1 (in radians): {joint_angle_1}")

print(f"Joint Angle 2: {np.degrees(joint_angle_2):.2f} degrees")
print(f"Joint Angle 2 (in radians): {joint_angle_2}")

print(f"Prismatic Displacement (d3): {prismatic_displacement:.2f} units")

calculated_end_effector_x, calculated_end_effector_y, calculated_end_effector_z = calculate_forward_kinematics(
    link1_length, link2_length, link3_length, joint_angle_1, joint_angle_2, prismatic_displacement
)

