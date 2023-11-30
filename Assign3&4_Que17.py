import numpy as np

def calculate_euler_angles(C, D):
    # Calculate the transformation matrix from the current to the desired orientation
    R_diff = np.dot(C.T, D) #rotation_matrix_difference = np.dot(current_orientation_matrix.T, desired_orientation_matrix)


    # Extract Z-Y-Z Euler angles from the rotation matrix
    phi = np.arctan2(R_diff[1, 2], R_diff[0, 2])

    # Ensure that the input to arccos is within the valid range [-1, 1]
    cos_theta = R_diff[2, 2]
    if np.abs(cos_theta) > 1:
        cos_theta = np.sign(cos_theta)  # Adjust to the nearest valid value
    theta = np.arccos(cos_theta)

    psi = np.arctan2(R_diff[2, 1], -R_diff[2, 0])

    return phi, theta, psi

if __name__ == "__main__":
    print("3D Printer Inverse Kinematics / Inverse kinematics of Spherical Wrist manipulator:\n")

    # Take user input for joint angles
    theta1_deg = float(input("Enter Joint 1 Angle (degrees): "))
    theta2_deg = float(input("Enter Joint 2 Angle (degrees): "))
    theta3_deg = float(input("Enter Joint 3 Angle (degrees): "))

    # Convert joint angles to radians for calculations
    theta1_rad = np.radians(theta1_deg)
    theta23_rad = np.radians(theta2_deg + theta3_deg)

    # Calculate cosine and sine values for efficiency
    c1 = np.cos(theta1_rad)
    c23 = np.cos(theta23_rad)
    s1 = np.sin(theta1_rad)
    s23 = np.sin(theta23_rad)

    # Define the rotation matrix of the current end-effector orientation
    C = np.array([[-c1 * c23, -c1 * s23, s1],
                  [s1 * c23, -s1 * s23, -c1],
                  [s23, c23, 0]])

    # Take user input for the desired end-effector orientation
    print("\nEnter the elements of the desired end-effector orientation matrix (row by row):")
    D = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            D[i, j] = float(input(f"Desired_orientation_matrix[{i}][{j}]: "))

    # Calculate Z-Y-Z Euler angles
    phi, theta, psi = calculate_euler_angles(C, D)

    # Print the results in a user-friendly format
    print("\nDesired Orientation Matrix:")
    print(D)
    print("\nCurrent Orientation Matrix:")
    print(C)
    print("\nCalculated Z-Y-Z Euler Angles:")
    print("Phi =", np.degrees(phi), "degrees")
    print("Theta =", np.degrees(theta), "degrees")
    print("Psi =", np.degrees(psi), "degrees")
