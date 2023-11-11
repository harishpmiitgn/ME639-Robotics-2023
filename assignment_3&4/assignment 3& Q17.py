import numpy as np

def calculate_euler_angles(current_orientation, desired_orientation):
    # Calculate the transformation matrix that transforms from the current to the desired orientation
    R_d2c = np.dot(current_orientation.T, desired_orientation)

    # Extract the z-y-z Euler angles from the rotation matrix
    phi = np.arctan2(R_d2c[1, 2], R_d2c[0, 2])
    theta = np.arccos(R_d2c[2, 2])
    psi = np.arctan2(R_d2c[2, 1], -R_d2c[2, 0])

    return phi, theta, psi

if __name__ == "__main__":
    print("Inverse kinematics of Spherical Wrist manipulator:\n")

    # Take input for joint angles
    joint1_angle_deg = float(input("Enter angle of Joint 1 (deg): "))
    joint2_angle_deg = float(input("Enter angle of Joint 2 (deg): "))
    joint3_angle_deg = float(input("Enter angle of Joint 3 (deg): "))

    c1 = np.cos(np.radians(joint1_angle_deg))
    c23 = np.cos(np.radians(joint2_angle_deg + joint3_angle_deg))
    s1 = np.sin(np.radians(joint1_angle_deg))
    s23 = np.sin(np.radians(joint2_angle_deg + joint3_angle_deg))

    # Define the rotation matrix of the current end-effector orientation
    R_current = np.array([[-c1 * c23, -c1 * s23, s1],
                          [s1 * c23, -s1 * s23, -c1],
                          [s23, c23, 0]])

    # Take input of desired end-effector orientation
    print("\nEnter the elements of the desired end-effector orientation matrix (row by row):")
    R_desired = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            R_desired[i, j] = float(input(f"R_desired[{i}][{j}]: "))

    phi, theta, psi = calculate_euler_angles(R_current, R_desired)

    # Print the results
    print("\nDesired Rotation Matrix:")
    print(R_desired)
    print("\nCurrent Rotation Matrix:")
    print(R_current)
    print("\nCalculated Z-Y-Z Euler Angles:")
    print("phi =", phi * (180 / np.pi), "deg")
    print("theta =", theta * (180 / np.pi), "deg")
    print("psi =", psi * (180 / np.pi), "deg")
