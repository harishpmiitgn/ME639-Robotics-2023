import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Manipulator parameters
L1 = 1.0  # Length of the first link
L2 = 1.0  # Length of the second link

# Trajectory parameters
num_points = 100
start_point = np.array([1.3, 0.5])  # Starting position (x, y)
end_point = np.array([-1.5, -0.3])  # Ending position (x, y)
trajectory = np.linspace(start_point, end_point, num_points)

 
def inverse_kinematics(x, y):
    theta2 = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    theta1 = np.arctan2(y,x) - np.arctan2(L2 * np.sin(theta2),( L1 + L2 * np.cos(theta2)))
    return theta1, theta2

# Initialize plot
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
line, = ax.plot([], [], 'o-', lw=2)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    target_point = trajectory[i]
    target_x, target_y = target_point

    theta1, theta2 = inverse_kinematics(target_x, target_y)
    manipulator_x = [0, L1 * np.cos(theta1), L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)]
    manipulator_y = [0, L1 * np.sin(theta1), L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)]

    line.set_data(manipulator_x, manipulator_y)
    return line,

# Animate the manipulator following the trajectory
ani = FuncAnimation(fig, animate, frames=len(trajectory), init_func=init, blit=True)
plt.show()
