import numpy as np

def rotation_matrix(axis, angle):
    c = np.cos(angle)
    s = np.sin(angle)
    
    if axis == 'x':
        return np.array([[1, 0, 0],
                         [0, c, -s],
                         [0, s, c]])
    elif axis == 'y':
        return np.array([[c, 0, s],
                         [0, 1, 0],
                         [-s, 0, c]])
    elif axis == 'z':
        return np.array([[c, -s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
    else:
        raise ValueError("Invalid rotation axis")

def stanford(theta1, theta2, d3):
    a1 = 1.0  # Length of the first link
    a2 = 1.0  # Length of the second link
    
    # Create the transformation matrices for each joint
    T1 = np.eye(4)
    T1[:3, :3] =rotation_matrix('z', theta1)
    
    T2 = np.eye(4)
    T2[:3, :3] =rotation_matrix('x', theta2)
    
    T3 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, d3],
                   [0, 0, 0, 1]])
    
    # Calculate the transformation matrix for the end effector
    T_end_effector = np.dot(np.dot(T1, T2), T3)
    
    # Extract the position vector from the transformation matrix
    position_vector = T_end_effector[:3, 3]
    
    return T_end_effector, position_vector

# Example joint variables
theta1 = np.radians(45)  # Angle first link
theta2 = np.radians(-30)  # Angle second link
d3 = 2.0  # Prismatic joint displacement

T_end_effector, position_vector = stanford(theta1, theta2, d3)

print("Transformation Matrix:")
print(T_end_effector)

print("\nPosition Vector of End Effector:")
print(position_vector)
