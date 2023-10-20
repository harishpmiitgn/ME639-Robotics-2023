import numpy as np

def compute_joint_velocities(J, X_dot):

    # Compute the inverse of the Jacobian
    J_inv = np.linalg.pinv(J)

    # Compute joint velocities
    q_dot = np.dot(J_inv, X_dot)

    return q_dot

