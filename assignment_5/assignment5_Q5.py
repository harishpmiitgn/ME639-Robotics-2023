import numpy as np

def inverse_kinematics_spherical_wrist():
    """
    Performs inverse kinematics for a Spherical Wrist manipulator.
    """
    print("Inverse kinematics of Spherical Wrist manipulator:\n")

    # Take input for joint angles
    joint_angle_1 = np.radians(float(input("Enter angle of Joint 1 (degrees): ")))
    joint_angle_2 = np.radians(float(input("Enter angle of Joint 2 (degrees): ")))
    joint_angle_3 = np.radians(float(input("Enter angle of Joint 3 (degrees): ")))

    # Take input of desired end-effector orientation
    print("\nEnter the elements of the desired end-effector orientation matrix (row by row):")
    rotation_desired = np.empty((3, 3))
    for i in range(3):
        for j in range(3):
            rotation_desired[i][j] = float(input(f"R_desired[{i}][{j}]: "))

    # Define the rotation matrix of the current end-effector orientation
    cos_joint_angle_1 = np.cos(joint_angle_1)
    cos_joint_angles_23 = np.cos(joint_angle_2 + joint_angle_3)
    sin_joint_angle_1 = np.sin(joint_angle_1)
    sin_joint_angles_23 = np.sin(joint_angle_2 + joint_angle_3)
    
    rotation_current = np.array([[-cos_joint_angle_1 * cos_joint_angles_23, -cos_joint_angle_1 * sin_joint_angles_23, sin_joint_angle_1],
                                [sin_joint_angle_1 * cos_joint_angles_23, -sin_joint_angle_1 * sin_joint_angles_23, -cos_joint_angle_1],
                                [sin_joint_angles_23, cos_joint_angles_23, 0]])

    # Calculate the transformation matrix that transforms from the current to the desired orientation
    rotation_current_to_desired = np.dot(rotation_current.T, rotation_desired)

    # Extract the z-y-z Euler angles from the rotation matrix
    phi = np.arctan2(rotation_current_to_desired[1, 2], rotation_current_to_desired[0, 2])
    theta = np.arccos(rotation_current_to_desired[2, 2])
    psi = np.arctan2(rotation_current_to_desired[2, 1], -rotation_current_to_desired[2, 0])

    # Print the results
    print("\nDesired Orientation Matrix:")
    print(rotation_desired)
    print("\nCurrent Orientation Matrix:")
    print(rotation_current)
    print("\nCalculated Z-Y-Z Euler Angles:")
    print("Roll (phi) =", np.degrees(phi), "degrees")
    print("Pitch (theta) =", np.degrees(theta), "degrees")
    print("Yaw (psi) =", np.degrees(psi), "degrees")

# Perform inverse kinematics for Spherical Wrist manipulator
inverse_kinematics_spherical_wrist()
