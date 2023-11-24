import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 0.25
l2 = 0.25
l3 = 0.25

# coordinates for points A, B, C, D
A = (0.40, 0.06, 0.1)
B = (0.40, 0.01, 0.1)
C = (0.35, 0.01, 0.1)
D = (0.35, 0.06, 0.1)

# Create a trajectory for the square ABCD
waypoints = np.array([A, B, C, D, A])

# forward kinematics
def forward_kinematics(theta1, theta2, theta3):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
    z = 0.1  

    return x, y, z

# inverse kinematics
def inverse_kinematics(x, y, z):
    r = np.sqrt(x**2 + y**2)
    phi = np.arccos((l1**2 + l2**2 - r**2) / (2 * l1 * l2))
    theta2 = np.pi - phi
    alpha = np.arccos((r**2 + l1**2 - l2**2) / (2 * l1 * r))
    beta = np.arctan2(y, x)
    theta1 = beta - alpha
    theta3 = np.pi - theta1 - theta2

    return theta1, theta2, theta3

# Function to update the plot for each frame
def update(frame, sc, line):
    x, y, z = waypoints[frame]

    sc._offsets3d = (waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])

    line.set_data(waypoints[:frame + 1, 0], waypoints[:frame + 1, 1])
    line.set_3d_properties(waypoints[:frame + 1, 2])

    return sc, line

# Plot 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([0, 0.5])
ax.set_ylim([0, 0.1])
ax.set_zlim([0, 0.25])

colors = ['red', 'green', 'blue', 'purple', 'red']
sc = ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c=colors, marker='o')

# Line to connect waypoints
line, = ax.plot([], [], [], 'r-', lw=1)

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('PUMA-style RRR Robot Trajectory for Square ABCD')

ani = FuncAnimation(fig, update, frames=len(waypoints), fargs=(sc, line), blit=True)

plt.show()
