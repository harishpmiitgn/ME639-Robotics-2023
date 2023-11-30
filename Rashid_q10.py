import numpy as np

def jacobian(theta1, theta2, theta3):
    # Define link lengths
    L1 = 1
    L2 = 1
    L3 = 1

    # Define transformation matrices
    T1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                   [np.sin(theta1), np.cos(theta1), 0, 0],
                   [0, 0, 1, L1],
                   [0, 0, 0, 1]])
    T2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, L2],
                   [np.sin(theta2), np.cos(theta2), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    T3 = np.array([[np.cos(theta3), -np.sin(theta3), 0, L3],
                   [np.sin(theta3), np.cos(theta3), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    # Calculate end effector position
    T = T1 @ T2 @ T3
    o3 = T[:3,-1]

    # Define axes of rotation
    z0 = np.array([0, 0, 1])
    z1 = z0
    z2 = z0

    # Define origins of frames
    o0 = np.array([0, 0, 0])
    o1 = T1[:3,-1]
    o2 = (T1 @ T2)[:3,-1]

    # Calculate linear velocity part of Jacobian
    Jv = np.zeros((3,3))
    Jv[:,0] = np.cross(z0,o3 - o0)
    Jv[:,1] = np.cross(z1,o3 - o1)
    Jv[:,2] = np.cross(z2,o3 - o2)

    # Calculate angular velocity part of Jacobian
    Jw = np.zeros((3,3))
    Jw[:,0] = z0
    Jw[:,1] = z1
    Jw[:,2] = z2

    # Combine to form full Jacobian
    J = np.vstack((Jv,Jw))

    return J

# Example usage
theta1_val = np.pi/4
theta2_val = np.pi/4
theta3_val = np.pi/4
J_val = jacobian(theta1_val, theta2_val, theta3_val)
print(J_val)
