import numpy as np

def transformation_matrix(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def manipulator_kinematics(DH_parameters, joint_velocities=None, joint_types=None):

    num_links = DH_parameters.shape[0]

    # If joint types are not provided, assume all are revolute joints
    if joint_types is None:
        joint_types = ['R'] * num_links

    # Initialize the total transformation matrix and Jacobian
    T = np.eye(4)
    J = np.zeros((6, num_links))

    for i in range(num_links):
        alpha, a, d, theta = DH_parameters[i]

        # Update the total transformation matrix
        T_i = transformation_matrix(alpha, a, d, theta)
        T = np.matmul(T, T_i)

        # Compute the Jacobian column for this joint
        z_prev = np.array([0, 0, 1]) if i == 0 else T_prev[:3, 2]
        p_end_effector = T[:3, 3]
        p_joint = np.array([0, 0, 0]) if i == 0 else T_prev[:3, 3]

        if joint_types[i] == 'R':  # Revolute joint
            J[:3, i] = np.cross(z_prev, p_end_effector - p_joint)
            J[3:, i] = z_prev
        else:  # Prismatic joint
            J[:3, i] = z_prev
            J[3:, i] = np.array([0, 0, 0])

        T_prev = T.copy()

    # Compute the end-effector velocity if joint velocities are provided
    v_end_effector = np.dot(J, joint_velocities) if joint_velocities is not None else None

    return J, T[:3, 3], v_end_effector


# Example DH parameters for RRP configuration of Stanford manipulator
DH_parameters = np.array([
    [0, 0, 0, np.pi/6],         # Revolute Joint (30 degrees)
    [np.pi/2, 1, 0, np.pi/4],   # Revolute Joint (45 degrees)
    [0, 1, 0.5, 0]              # Prismatic Joint
])
joint_velocities = np.array([1, 2, 1])


J, ef_pos, ef_vel = manipulator_kinematics(DH_parameters, joint_velocities)

print(J, ef_pos, ef_vel)

# Example DH parameters for RRP configuration of SCARA manipulator
DH_parameters = np.array([
    [0, 0, 0, np.pi/6],         # Revolute Joint (30 degrees)
    [0, 1, 0, np.pi/4],         # Revolute Joint (45 degrees)
    [0, 1, 0.5, 0]              # Prismatic Joint
])
joint_velocities = np.array([1, 2, 1])

J, ef_pos, ef_vel = manipulator_kinematics(DH_parameters, joint_velocities)

print(J, ef_pos, ef_vel)

