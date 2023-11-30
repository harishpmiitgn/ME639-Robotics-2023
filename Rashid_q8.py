import numpy as np
import sympy as sp

def jacobian(theta1, theta2, d3):
    # Define link lengths
    L1 = 1
    L2 = 1

    # Define joint variables
    theta1, theta2, d3 = sp.symbols('theta1 theta2 d3')

    # Define transformation matrices
    T1 = sp.Matrix([[sp.cos(theta1), -sp.sin(theta1), 0, 0],
                    [sp.sin(theta1), sp.cos(theta1), 0, 0],
                    [0, 0, 1, L1],
                    [0, 0, 0, 1]])
    T2 = sp.Matrix([[sp.cos(theta2), -sp.sin(theta2), 0, L2],
                    [sp.sin(theta2), sp.cos(theta2), 0, 0],
                    [0, 0, 1, d3],
                    [0, 0, 0, 1]])

    # Calculate end effector position
    T = T1 @ T2
    o3 = T[:3,-1]

    # Define axes of rotation
    z0 = sp.Matrix([0, 0, 1])
    z1 = T1[:3,-2]
    z2 = sp.Matrix([0, 0, 1])

    # Define origins of frames
    o0 = sp.Matrix([0, 0, 0])
    o1 = T1[:3,-1]

    # Calculate linear velocity part of Jacobian
    Jv = sp.zeros(3)
    Jv[:,0] = z0.cross(o3 - o0)
    Jv[:,1] = z1.cross(o3 - o1)
    Jv[:,2] = z2

    # Calculate angular velocity part of Jacobian
    Jw = sp.zeros(3)
    Jw[:,0] = z0
    Jw[:,1] = z1

    # Combine to form full Jacobian
    J = sp.vstack(Jv,Jw)

    return J

# Example usage
theta1_val = np.pi/4
theta2_val = np.pi/4
d3_val = 0.5
J_val = jacobian(theta1_val, theta2_val, d3_val)
print(J_val)
