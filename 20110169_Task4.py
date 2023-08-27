import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
l1 = 0.6
l2 = 0.3

# Joint angle ranges (in radians)
q1_range = np.radians(np.array([35, 145]))
q2_range = np.radians(np.array([35, 145]))

# Initialize variables for workspace
workspace_x = []
workspace_y = []

# Iterate through joint angles and calculate end effector positions
for q1 in np.linspace(q1_range[0], q1_range[1], 100):
    for q2 in np.linspace(q2_range[0], q2_range[1], 100):
        x_end = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
        y_end = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
        workspace_x.append(x_end)
        workspace_y.append(y_end)

# Plot the workspace
plt.figure(figsize=(8, 6))
plt.scatter(workspace_x, workspace_y, s=5, c='blue', label='Workspace')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Workspace of 2R Manipulator')
plt.grid(True)
plt.legend()
plt.show()
