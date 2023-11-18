import numpy as np

def inverse_kinematics_spherical_wrist():

    # User input for the elements of the rotation matrix
    print("Enter the elements of the 3x3 rotation matrix:")
    R_elements = [[float(input(f'Enter element for R({i + 1}, {j + 1}): ')) for j in range(3)] for i in range(3)]
    
    # Convert the input to a NumPy array
    R = np.array(R_elements)

    # Extract rotation matrix elements
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]

    # Calculate joint angles
    theta1 = np.arctan2(r21, r11)
    theta2 = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))
    theta3 = np.arctan2(r32, r33)

    return [theta1, theta2, theta3], R

# Calculate joint angles and store input matrix
joint_angles, input_matrix = inverse_kinematics_spherical_wrist()

# Print the result
print("\n" + "-" * 50 + "\n")
print("Inverse Kinematics Result:")
print("\nInput Rotation Matrix:")
print(input_matrix)
print("\nJoint Angles (in radians):") 
print(joint_angles)

