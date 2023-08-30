import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import minimize

# Length of links
l1 = l2 = 1

# Initialize joint_angles array to store joint angles
# joint_angles[0] stores theta1 and joint_angles[1] stores theta2
# joint_angles[:, i] stores joint angles at ith time step
num_timesteps = 100
joint_angles = np.zeros((2, num_timesteps + 1))

# Y-Z coordinates of the desired point
desired_point = np.array([0.5, 1.5])

# Force applied at the end effector
force = np.array([0, 0.2])

# Function to compute the position of the end effector given joint angles
def compute_end_effector_position(theta1, theta2):
    z = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return y, z

# Objective function to minimize the distance between the end effector and the desired point
def objective_function(theta):
    y, z = compute_end_effector_position(theta[0], theta[1])
    distance = np.sqrt((y - desired_point[0]) ** 2 + (z - desired_point[1]) ** 2)
    return distance

# Constraints on joint angles
constraints = (
    {'type': 'ineq', 'fun': lambda x: x[0]},
    {'type': 'ineq', 'fun': lambda x: -np.pi / 2 - x[0]},
    {'type': 'ineq', 'fun': lambda x: x[1] - (-np.pi / 2)},
    {'type': 'ineq', 'fun': lambda x: np.pi - x[1]}
)

# Find the optimal joint angles using the minimize function
result = minimize(objective_function, [0, 0], method='COBYLA', constraints=constraints)

# Store the optimal joint angles in the joint_angles array
joint_angles[:, 0] = result.x

# Compute the joint angles for the rest of the time steps
for i in range(num_timesteps):
    theta1, theta2 = joint_angles[:, i]
    y, z = compute_end_effector_position(theta1, theta2)
    jacobian = np.array([[-l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2), -l2 * np.sin(theta1 + theta2)],
                         [l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2), l2 * np.cos(theta1 + theta2)]])
    dx, dy = desired_point - np.array([y, z])
    dtheta = np.linalg.pinv(jacobian) @ np.array([dy, dx])

    # Apply a force at the end effector
    mass = 1
    acceleration = force / mass
    dtheta += np.linalg.pinv(jacobian) @ (acceleration * (1 / l2))

    joint_angles[:, i + 1] = joint_angles[:, i] + dtheta

# Create figure and axis
fig = plt.figure()
axis = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-1.5, 2.5), ylim=(-1, 2))

# Initialize arm segments
link1, = axis.plot([], [], 'bo-', lw=2)
link2, = axis.plot([], [], 'ro-', lw=2)

def init():
    link1.set_data([], [])
    link2.set_data([], [])
    return link1, link2

def animate(i):
    theta1, theta2 = joint_angles[:, i]

    # Compute arm segment coordinates
    x0, y0 = 0, 0
    x1, y1 = x0 + l1 * np.cos(theta1), y0 + l1 * np.sin(theta1)
    x2, y2 = x1 + l2 * np.cos(theta1 + theta2), y1 + l2 * np.sin(theta1 + theta2)

    # Update arm segments
    link1.set_data([x0, x1], [y0, y1])
    link2.set_data([x1, x2], [y1, y2])

    # Apply a force at the end effector
    if i == num_timesteps:
        link2.set_linestyle('--')  # Change the linestyle of the second link to dashed to indicate force application

    return link1, link2

# Set up animation
ani = animation.FuncAnimation(fig, animate, frames=num_timesteps + 1, init_func=init, interval=200, blit=True)

# Display animation
plt.show()

# Create figure and axis
fig = plt.figure()
axis = fig.add_subplot(111, projection='3d')

# Initialize arm segments
link1, = axis.plot([], [], [], 'bo-', lw=2)
link2, = axis.plot([], [], [], 'ro-', lw=2)

def init():
    link1.set_data([], [])
    link1.set_3d_properties([])
    link2.set_data([], [])
    link2.set_3d_properties([])
    return link1, link2

def animate(i):
    theta1, theta2 = joint_angles[:, i]

    # Compute arm segment coordinates
    x0, y0, z0 = 0, 0, 0
    x1, y1, z1 = x0 + l1 * np.cos(theta1), y0 + l1 * np.sin(theta1), z0
    x2, y2, z2 = x1 + l2 * np.cos(theta1 + theta2), y1 + l2 * np.sin(theta1 + theta2), z1

    # Update arm segments
    link1.set_data([x0, x1], [y0, y1])
    link1.set_3d_properties([z0, z1])
    link2.set_data([x1, x2], [y1, y2])
    link2.set_3d_properties([z1, z2])
    return link1, link2

# Set the limits of the 3D plot
axis.set_xlim(-2, 2)
axis.set_ylim(-2, 2)
axis.set_zlim(-2, 2)

# Set labels and title
axis.set_xlabel('X')
axis.set_ylabel('Y')
axis.set_zlabel('Z')
axis.set_title('2D Arm Animation')

# Create the animation
ani = animation.FuncAnimation(fig, animate, frames=num_timesteps, init_func=init, blit=True)

# Display the animation
plt.show()

