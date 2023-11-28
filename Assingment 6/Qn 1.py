import numpy as np
import matplotlib.pyplot as plt

l1 = 1
l2 = 1
l3 = 1
r = 1.5
center = np.array([0, 0])

# calculate forward kinematics
def forward_kinematics(theta):
    x = l1 * np.cos(theta[0]) + l2 * np.cos(theta[0] + theta[1]) + l3 * np.cos(np.sum(theta))
    y = l1 * np.sin(theta[0]) + l2 * np.sin(theta[0] + theta[1]) + l3 * np.sin(np.sum(theta))
    return np.array([x, y])

# calculate Jacobian matrix
def jacobian(theta):
    J = np.zeros((2, 3))
    J[0, 0] = -l1 * np.sin(theta[0]) - l2 * np.sin(theta[0] + theta[1]) - l3 * np.sin(np.sum(theta))
    J[0, 1] = -l2 * np.sin(theta[0] + theta[1]) - l3 * np.sin(np.sum(theta))
    J[0, 2] = -l3 * np.sin(np.sum(theta))
    J[1, 0] = l1 * np.cos(theta[0]) + l2 * np.cos(theta[0] + theta[1]) + l3 * np.cos(np.sum(theta))
    J[1, 1] = l2 * np.cos(theta[0] + theta[1]) + l3 * np.cos(np.sum(theta))
    J[1, 2] = l3 * np.cos(np.sum(theta))
    return J

# inverse kinematics
def inverse_kinematics(theta_init, target_position, max_iterations=100, tolerance=1e-6, alpha=0.1):
    theta = theta_init.copy()

    for i in range(max_iterations):
        end_effector = forward_kinematics(theta)
        error = target_position - end_effector
        J = jacobian(theta)
        delta_theta = alpha * np.dot(J.T, error)

        theta += delta_theta

        if np.linalg.norm(error) < tolerance:
            print(f"Converged in {i+1} iterations.\n")

            return theta

    return None

# Initial guess for joint angles
theta_init = np.array([0.5, 0.25, 0.25])

# Desired circle points
t_vals = np.linspace(0, 2 * np.pi, 100)
circle_points = np.array([center + r * np.array([np.cos(t), np.sin(t)]) for t in t_vals])

joint_angles = []
for target_point in circle_points:
    result = inverse_kinematics(theta_init, target_point)
    if result is not None:
        joint_angles.append(result)

# Plotting the results
joint_angles = np.array(joint_angles)
plt.plot(circle_points[:, 0], circle_points[:, 1], label='Desired Circle')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Inverse Kinematics for 3R Manipulator')
plt.legend()
plt.show()
