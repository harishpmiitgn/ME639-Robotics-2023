import numpy as np

#alpha 1 and alpha 2 are 0 for 2R planar manipulator
def dh_transform_matrix(theta, d, a, alpha):
    """
    Calculate the transformation matrix for a single joint.

    theta: Joint angle in radians.
    :param d: Link offset.
    :param a: Link length.
    :param alpha: Link twist.
    :return: Transformation matrix for the joint.
    """
    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics_2r(theta1, theta2, d1, a1, d2, a2,alpha1,alpha2):
    """
    Calculate the transformation matrix of the end effector for a 2R planar manipulator.

    theta1: Joint 1 angle in radians.
    theta2: Joint 2 angle in radians.
    d1: Link 1 offset.
    a1: Link 1 length.
    d2: Link 2 offset.
    a2: Link 2 length.
    
    """
    T1 = dh_transform_matrix(theta1, d1, a1, alpha1)
    T2 = dh_transform_matrix(theta2, d2, a2, alpha2)

    T_end_effector = np.dot(T1, T2)

    return T_end_effector

def jacobian_2r(theta1, theta2, d1, a1, d2, a2,alpha1,alpha2):
    """
    Calculate the Jacobian matrix for a 2R planar manipulator and end effector position.
    theta1: Joint 1 angle in radians.
    theta2: Joint 2 angle in radians.
    d1: Link 1 offset.
    a1: Link 1 length.
    d2: Link 2 offset.
    a2: Link 2 length.
    :return: Jacobian matrix.
    """
    
    T2 = forward_kinematics_2r(theta1,d2,a2,alpha1,theta2, d2, a2, alpha2)

    # End effector position
    x = T2[0, 3]
    y = T2[1, 3]

    # Jacobian matrix
    J = np.array([
        [-a1 * np.sin(theta1) - a2 * np.sin(theta1 + theta2), -a2 * np.sin(theta1 + theta2)],
        [a1 * np.cos(theta1) + a2 * np.cos(theta1 + theta2), a2 * np.cos(theta1 + theta2)],
        [0, 0]
    ])

    return J, (x, y)


theta1 = np.pi / 4  # Joint 1 angle
theta2 = np.pi / 3  # Joint 2 angle
d1 = 0              # Link 1 offset
a1 = 91.88              # Link 1 length
d2 = 0              # Link 2 offset
a2 = 104.54              # Link 2 length
alpha1=0            # Link 1 twist
alpha2=0            # Link 2 twist

T_end_effector = forward_kinematics_2r(theta1, theta2, d1, a1, d2, a2,alpha1,alpha2)
print("Transformation Matrix of End Effector:")
print(T_end_effector)

Jacobian, end_effector_position = jacobian_2r(theta1, theta2, d1, a1, d2, a2,alpha1,alpha2)
print("Jacobian Matrix:")
print(Jacobian)

print("End Effector Position:")
print(f"X: {end_effector_position[0]}")
print(f"Y: {end_effector_position[1]}")