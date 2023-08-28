import numpy as np
import matplotlib.pyplot as plt

# Parameters
l1 = 1  # length of arm 1
l2 = 1  # length of arm 2
theta1_range = np.deg2rad(np.linspace(35, 145, 100))  # Joint angle 1 range
theta2_range = np.deg2rad(np.linspace(35, 145, 100))  # Joint angle 2 range

# Forward kinematics function
def forward_kinematics(theta1, theta2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

# Calculate workspace points
workspace_points = []
for theta1 in theta1_range:
    for theta2 in theta2_range:
        x, y = forward_kinematics(theta1, theta2)
        workspace_points.append((x, y))
workspace_points = np.array(workspace_points)

# Plot workspace
plt.figure(figsize=(8, 8))
plt.scatter(workspace_points[:, 0], workspace_points[:, 1], s=5, c='b', marker='.', label='Workspace')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Workspace of 2R Elbow Manipulator')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
