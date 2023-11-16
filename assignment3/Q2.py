import numpy as np

def dh_transform_matrix(theta, d, a, alpha):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    transformation_matrix = np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])

    return transformation_matrix
def end_effpos(dh_params):
    num_joints = len(dh_params)
    T = np.identity(4)
    for i in range(num_joints):
        print(T)
        theta, d, a, alpha = dh_params[i]
        T_i = dh_transform_matrix(theta, d, a, alpha)
        T = np.dot(T, T_i)
    print(T)
    return T[:3, 3]

def manipulator_jacobian(dh_params, joint_types=None):
    if joint_types is None:
        joint_types = ['R'] * len(dh_params)

    if len(dh_params) != len(joint_types):
        raise ValueError("Number of DH parameters and joint types must be the same.")

    num_joints = len(dh_params)
    T = np.identity(4)
    J = np.zeros((6, num_joints))
    end_effector_position =end_effpos(dh_params)
    for i in range(num_joints):
        theta, d, a, alpha = dh_params[i]
        T_i = dh_transform_matrix(theta, d, a, alpha)
        T = np.dot(T, T_i)
        if joint_types[i] == 'R':  # Revolute joint
            # Calculate the rotational part of the Jacobian
            z = T[:3, 2]
            p = end_effector_position-T[:3, 3]
            J[:3, i] = np.cross(z, p)

            # Calculate the translational part of the Jacobian
            J[3:, i] = z
        else:
            z = T[:3, 2]
            J[:3, i] = z

    
    end_effector_velocity =np.dot([0, 0, 0, 1, 1, 1],J)  # Default joint velocities

    return J, end_effector_position, end_effector_velocity

# Example usage: theta,d a,alpha
dh_params = [
    [0, 1, 0, 0],
    [-np.pi/2, 1, 0, -np.pi/2],
    [-np.pi/2, 1, 0, -np.pi/2],
    [0, 1, 0, 0]
]

joint_types = ["P","P","P","P"]

Jacobian, end_effector_position, end_effector_velocity = manipulator_jacobian(dh_params, joint_types)
print("Manipulator Jacobian:")
print(Jacobian)
print("End-effector Position:")
print(end_effector_position)
print("End-effector Velocity:")
print(end_effector_velocity)
