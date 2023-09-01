import numpy as np

def Jacobian(L1, L2, theta1, theta2, d3):
    dx_dtheta1 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    dx_dtheta2 = -L2 * np.sin(theta1 + theta2)
    dx_dd3 = 0.0

    dy_dtheta1 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    dy_dtheta2 = L2 * np.cos(theta1 + theta2)
    dy_dd3 = 0.0

    dz_dtheta1 = 0.0
    dz_dtheta2 = 0.0
    dz_dd3 = 1.0

    jacobian = np.array([[dx_dtheta1, dx_dtheta2, dx_dd3],
                         [dy_dtheta1, dy_dtheta2, dy_dd3],
                         [dz_dtheta1, dz_dtheta2, dz_dd3]])

    return jacobian

# Example usage
L1 = 1.0
L2 = 1.0
theta1 = np.pi / 4
theta2 = np.pi / 3
d3 = 0.5

jacobian = Jacobian(L1, L2, theta1, theta2, d3)
print("Scara RRP Manipulator Jacobian:")
print(jacobian)
