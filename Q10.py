import numpy as np

def Jacobian(theta1, theta2, theta3, lengths):
    l1, l2, l3 = lengths
    
    dx_dtheta1 = -l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2) - l3 * np.sin(theta1 + theta2 + theta3)
    dx_dtheta2 = -l2 * np.sin(theta1 + theta2) - l3 * np.sin(theta1 + theta2 + theta3)
    dx_dtheta3 = -l3 * np.sin(theta1 + theta2 + theta3)
    
    dy_dtheta1 = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    dy_dtheta2 = l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    dy_dtheta3 = l3 * np.cos(theta1 + theta2 + theta3)
    
    jacobian = np.array([[dx_dtheta1, dx_dtheta2, dx_dtheta3],
                         [dy_dtheta1, dy_dtheta2, dy_dtheta3]])
    
    return jacobian

# example data
theta1 = np.radians(20)
theta2 = np.radians(60)
theta3 = np.radians(-30)
lengths = [2.0, 1.5, 1.0] # Link lengths

# Compute the Jacobian
jacobian_matrix = Jacobian(theta1, theta2, theta3, lengths)
print(" RRR Elbow Manipulator Jacobian:")
print(jacobian_matrix)
