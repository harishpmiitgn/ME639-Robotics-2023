import numpy as np

def forward_kinematics(theta, d, a, alpha):
    # Compute the transformation matrix
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    T = np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])
    return T

def manipulator_jacobian(theta, d, a, alpha, end_effector_position):
    num_links = len(theta)
    jacobian = np.zeros((6, num_links))

    # Compute the forward kinematics for each joint and accumulate the transformations
    current_T = np.identity(4)
    for i in range(num_links):
        T_i = forward_kinematics(theta[i], d[i], a[i], alpha[i])
        current_T = np.dot(current_T, T_i)

        # Calculate the position of the current joint
        p_i = current_T[:3, 3]

        # Calculate the axis of rotation for the current joint
        z_i = current_T[:3, 2]

        # Calculate the cross product to determine the linear velocity component
        linear_velocity = np.cross(z_i, end_effector_position - p_i)

        # Calculate the angular velocity component
        angular_velocity = z_i

        # Fill in the columns of the Jacobian matrix
        jacobian[:, i] = np.concatenate((linear_velocity, angular_velocity))

    return jacobian

def end_effector_velocity(jacobian, joint_velocity):
    end_effector_velocity = np.dot(jacobian, joint_velocity)
    return end_effector_velocity

# Example usage:
num_links = 2
theta = [0.2, 0.4]  # Joint angles in radians
d = [1, 2]  # Link offsets (d-values)
a = [1, 1]  # Link lengths (a-values)
alpha = [0.4, .2]  # Link twist angles (alpha-values)
end_effector_position = np.array([3, 2, 1])  # Add a zero for the missing dimension

# Calculate the manipulator Jacobian
jacobian = manipulator_jacobian(theta, d, a, alpha, end_effector_position)

# Calculate the end-effector velocity
joint_velocity = np.array([0.1, 0.2])  # Joint velocities
end_effector_vel = end_effector_velocity(jacobian, joint_velocity)

print("Manipulator Jacobian:")
print(jacobian)
print("End-Effector Velocity:")
print(end_effector_vel)
print("\nassumed parameters: \n num_links = 2 \n theta = [0.2, 0.4] \n d = [1, 2] \n a = [1, 1] \n alpha = [0.4, .2] \n end_effector_position = np.array([3, 2, 1])")
