import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
dt = 0.03  # Time step
total_time = 10.0  # Total simulation time
l1 = 1      # length of arm 1
l2 = 1      # length of arm 2

# Desired trajectory function (example: circular trajectory)
def desired_trajectory(t):
    radius = 1.2
    omega = 0.8
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    return x, y

# Forward kinematics function
def forward_kinematics(theta1, theta2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

# Inverse kinematics function
def inverse_kinematics(x, y):
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    s2 = np.sqrt(1 - c2**2)
    theta2 = np.arctan2(s2, c2)
    theta1 = np.arctan2(y, x) - np.arctan2(l1 * s2, l1 + l2 * c2)
    return theta1, theta2

# Robot class
class Robot:
    def __init__(self):
        self.theta1 = 0.0
        self.theta2 = 0.0

    def update(self, desired_position):
        self.theta1, self.theta2 = inverse_kinematics(*desired_position)

# Simulation function
def simulate(robot_class, desired_trajectory):
    robot = robot_class()
    current_time = 0.0
    trajectory_points = []
    end_tip_positions = []

    while current_time <= total_time:
        desired_pos = np.array(desired_trajectory(current_time))
        robot.update(desired_pos)

        trajectory_points.append(desired_pos)
        end_tip_positions.append(forward_kinematics(robot.theta1, robot.theta2))

        current_time += dt

    return trajectory_points, end_tip_positions

# Simulate with manipulator
trajectory, end_tip_positions = simulate(Robot, desired_trajectory)

# Visualization with animations
fig, ax = plt.subplots(figsize=(6, 6))

ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_aspect('equal')
trajectory_line, = ax.plot([], [], 'b--', label='Desired Trajectory')
manipulator_line, = ax.plot([], [], 'mo-', label='Manipulator End-Tip') 
link1_line, = ax.plot([], [], 'g-', linewidth=4, label='Link 1')
link2_line, = ax.plot([], [], 'r-', linewidth=4, label='Link 2')
wall_line, = ax.plot([], [], 'k-', linewidth=7, label='Wall')  
ax.legend()
ax.set_title("2R Elbow Manipulator Trajectory Following")

# Wall parameters
wall_start_x = -1.0  # X-coordinate of the wall's starting point
wall_start_y = 0.5  # Y-coordinate of the wall's starting point
wall_length = 2.0  # Length of the wall

# Calculate the ending coordinates of the wall at a 45-degree angle
wall_end_x = wall_start_x + wall_length * np.cos(np.pi / 4)
wall_end_y = wall_start_y + wall_length * np.sin(np.pi / 4)

# Plot the wall
wall_line.set_data([wall_start_x, wall_end_x], [wall_start_y, wall_end_y])

def init_animation():
    trajectory_line.set_data([], [])
    manipulator_line.set_data([], [])
    link1_line.set_data([], [])
    link2_line.set_data([], [])
    wall_line.set_data([], [])  
    return trajectory_line, manipulator_line, link1_line, link2_line, wall_line

def animate(i):
    x_traj, y_traj = zip(*trajectory[:i+1])
    x_manipulator, y_manipulator = zip(*end_tip_positions[:i+1])

    theta1, theta2 = inverse_kinematics(x_manipulator[-1], y_manipulator[-1])
    x_link1 = [0, l1 * np.cos(theta1)]
    y_link1 = [0, l1 * np.sin(theta1)]
    x_link2 = [l1 * np.cos(theta1), x_manipulator[-1]]
    y_link2 = [l1 * np.sin(theta1), y_manipulator[-1]]

    trajectory_line.set_data(x_traj, y_traj)
    manipulator_line.set_data(x_manipulator[-1], y_manipulator[-1])
    link1_line.set_data(x_link1, y_link1)
    link2_line.set_data(x_link2, y_link2)

    # Update wall position
    wall_line.set_data([wall_start_x, wall_end_x], [wall_start_y, wall_end_y])

    # Check if end-tip is in contact with the wall
    distance_to_wall = abs(y_manipulator[-1] - wall_start_y - x_manipulator[-1] + wall_start_x)
    if distance_to_wall < 0.05:  
        ani.event_source.stop()  # Stop the animation

    return trajectory_line, manipulator_line, link1_line, link2_line, wall_line

ani = FuncAnimation(fig, animate, frames=len(trajectory), init_func=init_animation)

plt.tight_layout()
plt.show()
