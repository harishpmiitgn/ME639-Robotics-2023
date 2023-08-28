import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
dt = 0.03  # Time step
total_time = 10.0  # Total simulation time
l1 = 1  # length of arm 1
l2 = 1  # length of arm 2

# Forward kinematics function
def forward_kinematics(theta1, theta2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

# Inverse kinematics function
def inverse_kinematics(x, y):
    c2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    s2 = np.sqrt(1 - c2 ** 2)
    theta2 = np.arctan2(s2, c2)
    theta1 = np.arctan2(y, x) - np.arctan2(l1 * s2, l1 + l2 * c2)
    return theta1, theta2

# Robot class with control
class ControlledRobot:
    def __init__(self):
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.prev_error = np.zeros(2)

    def update(self, desired_position, time_step):
        current_position = np.array(forward_kinematics(self.theta1, self.theta2))
        error = np.array(desired_position) - current_position
        control_force = (
            spring_constant * (initial_deviation - error)
            - damping_coefficient * (error - self.prev_error) / time_step
        )
        new_desired_position = np.array(desired_position) + control_force
        self.theta1, self.theta2 = inverse_kinematics(*new_desired_position)
        self.prev_error = error

# Control parameters for the virtual spring
spring_constant = 5.0  # Adjust this value to control the stiffness of the virtual spring
damping_coefficient = 0.2  # Adjust this value to control damping

# Desired trajectory function (example: circular trajectory)
def desired_trajectory(t):
    radius = 1.2
    omega = 0.8
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    return x, y

# Calculate the mean position of xo and yo
robot = ControlledRobot()
trajectory, end_tip_positions = simulate(robot, desired_trajectory)
mean_xo = np.mean([point[0] for point in trajectory])
mean_yo = np.mean([point[1] for point in trajectory])
initial_deviation = np.array(
    [end_tip_positions[0][0] - mean_xo, end_tip_positions[0][1] - mean_yo]
)

# Simulation function
def simulate(robot, desired_trajectory):
    current_time = 0.0
    trajectory_points = []
    end_tip_positions = []

    while current_time <= total_time:
        desired_pos = np.array(desired_trajectory(current_time))
        robot.update(desired_pos, dt)

        trajectory_points.append(desired_pos)
        end_tip_positions.append(forward_kinematics(robot.theta1, robot.theta2))

        current_time += dt

    return trajectory_points, end_tip_positions

# Simulate with controlled robot
robot = ControlledRobot()
controlled_trajectory, controlled_end_tip_positions = simulate(
    robot, desired_trajectory
)

# Visualization with animations
fig, ax = plt.subplots(figsize=(6, 6))

ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_aspect("equal")
trajectory_line, = ax.plot([], [], "b--", label="Desired Trajectory")
manipulator_line, = ax.plot([], [], "mo-", label="Manipulator End-Tip")
link1_line, = ax.plot([], [], "g-", linewidth=4, label="Link 1")
link2_line, = ax.plot([], [], "r-", linewidth=4, label="Link 2")
ax.legend()
ax.set_title("2R Elbow Manipulator Trajectory Following with Virtual Spring Control")

def init_animation():
    trajectory_line.set_data([], [])
    manipulator_line.set_data([], [])
    link1_line.set_data([], [])
    link2_line.set_data([], [])
    return trajectory_line, manipulator_line, link1_line, link2_line

def animate(i):
    x_traj, y_traj = zip(*controlled_trajectory[:i + 1])
    x_manipulator, y_manipulator = zip(*controlled_end_tip_positions[:i + 1])

    theta1, theta2 = inverse_kinematics(x_manipulator[-1], y_manipulator[-1])
    x_link1 = [0, l1 * np.cos(theta1)]
    y_link1 = [0, l1 * np.sin(theta1)]
    x_link2 = [l1 * np.cos(theta1), x_manipulator[-1]]
    y_link2 = [l1 * np.sin(theta1), y_manipulator[-1]]

    trajectory_line.set_data(x_traj, y_traj)
    manipulator_line.set_data(x_manipulator[-1], y_manipulator[-1])
    link1_line.set_data(x_link1, y_link1)
    link2_line.set_data(x_link2, y_link2)

    return trajectory_line, manipulator_line, link1_line, link2_line

ani = FuncAnimation(
    fig,
    animate,
    frames=len(controlled_trajectory),
    init_func=init_animation,
    blit=True,
)

def on_click(event):
    if event.button == 1:  # Left mouse button
        desired_pos = np.array([event.xdata, event.ydata])
        robot.update(desired_pos, dt)
        controlled_trajectory.append(desired_pos)
        controlled_end_tip_positions.append(forward_kinematics(robot.theta1, robot.theta2))
        ani.frame_seq = range(len(controlled_trajectory))
        ani.event_source.stop()
        ani.event_source.start(interval=200)

fig.canvas.mpl_connect('button_press_event', on_click)

plt.tight_layout()
plt.show()
