import numpy as np

def calculate_joint_velocities(X_dot, J):
    n = J.shape[1]
    if n != 6:
        J_trans = J.T
        a = np.matmul(J_trans, J)
        b = np.linalg.inv(a)
        c = np.matmul(b, J_trans)
        q_dot = np.matmul(c, X_dot)
    else:
        J_inv = np.linalg.inv(J)
        q_dot = np.matmul(J_inv, X_dot)
    return q_dot