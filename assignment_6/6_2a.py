import numpy as np

# Define points A, B, C, D & link lengths & masses & moments of inertia
point_A = np.array([0.4, 0.06, 0.1])
point_B = np.array([0.4, 0.01, 0.1])
point_C = np.array([0.35, 0.01, 0.1])
point_D = np.array([0.35, 0.06, 0.1])
LINK_LENGTH_1 = 0.25
LINK_LENGTH_2 = 0.25
LINK_LENGTH_3 = 0.25
MASS_1 = 0.8
MASS_2 = 0.8
MASS_3 = 0.8
INERTIA_1 = 0.005
INERTIA_2 = 0.005
INERTIA_3 = 0.005

def scara_inverse_kinematics(x, y, z):
    """
    Calculate SCARA robot inverse kinematics.

    Parameters:
    - x, y, z: End effector position coordinates.

    Returns:
    - theta1, theta2, d3: Joint angles for SCARA robot.
    """
    theta2 = np.arccos((x**2 + y**2 - LINK_LENGTH_1**2 - LINK_LENGTH_2**2) / (2 * LINK_LENGTH_1 * LINK_LENGTH_2))

    # Calculate theta1 with adjustment
    theta1 = np.arctan2(y, x) - np.arctan2(LINK_LENGTH_2 * np.sin(theta2), LINK_LENGTH_1 + LINK_LENGTH_2 * np.cos(theta2))

    # Adjust theta1 to be within the range [-pi, pi]
    theta1 = (theta1 + np.pi) % (2 * np.pi) - np.pi
    d3 = -z

    return theta1, theta2, d3

def forward_kinematics(q):
    """
    Calculate SCARA robot forward kinematics.

    Parameters:
    - q: Joint angles for SCARA robot.

    Returns:
    - x, y, z: End effector position coordinates.
    """
    theta1, theta2, d3 = q
    T03 = build_dh_matrix(0, theta1, 0, LINK_LENGTH_1) @ build_dh_matrix(0, theta2, np.pi, LINK_LENGTH_2) @ build_dh_matrix(d3, 0, 0, 0)
    x, y, z = T03[:3, 3]
    return x, y, z

def build_dh_matrix(d, theta, alpha, a):
    """
    Build Denavit-Hartenberg transformation matrix.

    Parameters:
    - d, theta, alpha, a: DH parameters.

    Returns:
    - dh: Transformation matrix.
    """
    c_theta, s_theta = np.cos(theta), np.sin(theta)
    c_alpha, s_alpha = np.cos(alpha), np.sin(alpha)

    dh = np.array([
        [c_theta, -s_theta * c_alpha, s_theta * s_alpha, a * c_theta],
        [s_theta, c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
        [0, s_alpha, c_alpha, d],
        [0, 0, 0, 1]
    ])

    return dh

# Print results for each point
points = {'A': point_A, 'B': point_B, 'C': point_C, 'D': point_D}
for name, point in points.items():
    print(f"For {name}")
    
    # Calculate joint angles using inverse kinematics
    joint_angles = scara_inverse_kinematics(*point)
    print("Joint Angles:", joint_angles)
    
    # Calculate end effector position using forward kinematics
    end_effector_position = forward_kinematics(joint_angles)
    print("End Effector Position:", end_effector_position)
    print()
