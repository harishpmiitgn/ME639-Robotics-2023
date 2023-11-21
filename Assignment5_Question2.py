import numpy as np

def compute_manipulator_jacobian(theta, d, a, alpha, theta_dot):
    num_links = len(theta)
    
    # Initialize transformation matrices
    T_matrices = []
    for i in range(num_links):
        T = np.array([[np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],
                      [np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.cos(theta[i])*np.sin(alpha[i]), a[i]*np.sin(theta[i])],
                      [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        T_matrices.append(T)

    # Initialize the Jacobian matrix
    J = np.zeros((6, num_links))

    # Initialize the end-effector position
    end_effector_position = np.zeros(3)

    # Initialize the rotation matrix
    R = np.eye(3)

    for i in range(num_links):
        # Extract the rotation matrix from the homogeneous transformation
        R = R.dot(T_matrices[i][:3, :3])

        # Extract the position components (x, y, z) of the end-effector
        end_effector_position = T_matrices[i][:3, 3]

        # Compute the angular velocity Jacobian matrix (Jω) with respect to the joint frames
        Jw = np.transpose(R).dot(np.array([0, 0, 1]))

        # Compute the linear velocity Jacobian matrix (Jv) with respect to the joint frames
        Jv = np.transpose(R).dot(np.array([1, 0, 0]))

        # Combine Jv and Jω to get the full Jacobian matrix (J)
        J[:, i] = np.concatenate((Jv, Jw))

    # Compute end-effector velocity
    end_effector_velocity = np.dot(J, theta_dot)

    return J, end_effector_position, end_effector_velocity

def compute_joint_velocities(theta, d, a, alpha, end_effector_velocity):
    J, _, _ = compute_manipulator_jacobian(theta, d, a, alpha, np.zeros(len(theta)))
    
    # Calculate joint velocities using the inverse Jacobian relationship
    J_inv = np.linalg.pinv(J)  # Pseudo-inverse of J
    joint_velocities = np.dot(J_inv, end_effector_velocity)
    
    return joint_velocities

# Example usage
theta = np.array([0.1, 0.2, 0]) 
d = np.array([0, 0, 0.6])  
a = np.array([0.7, 0.8, 0.9])  
alpha = np.array([0, 0, 0])  

end_effector_velocity = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])  # Replace with actual end-effector velocities

joint_velocities = compute_joint_velocities(theta, d, a, alpha, end_effector_velocity)

print("Joint Velocities:")
print(joint_velocities)
