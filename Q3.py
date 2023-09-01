import numpy as np

def scara_position(theta1, theta2, d3, a1, a2):
    # Transformation matrices
    T1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, a1*np.cos(theta1)],
                   [np.sin(theta1), np.cos(theta1), 0, a1*np.sin(theta1)],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    
    T2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, a2*np.cos(theta2)],
                   [np.sin(theta2), np.cos(theta2), 0, a2*np.sin(theta2)],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    
    T3 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, d3],
                   [0, 0, 0, 1]])
    
    # Final transformation matrix
    T = np.dot(np.dot(T1, T2), T3)
    
    # End effector position vector
    position = T[:2, 3]
    
    return position

# Example joint variables
theta1 = np.radians(30)  # Joint angle 1
theta2 = np.radians(45)  # Joint angle 2
d3 = 10.0                # Joint distance 3
a1 = 15.0                # Link length 1
a2 = 10.0                # Link length 2

# Calculate end effector position
end_effector_position = scara_position(theta1, theta2, d3, a1, a2)
print("End Effector Position:", end_effector_position)
