import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 1
l2 = 1
l3 = 1

# calculate end effector position
def forward_kinematics(theta1, theta2, theta3):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
    return x, y

# inverse kinematics
def inverse_kinematics(x, y):
    r = np.sqrt(x**2 + y**2)
    phi = np.arccos((l1**2 + l2**2 - r**2) / (2 * l1 * l2))
    theta2 = np.pi - phi
    alpha = np.arccos((r**2 + l1**2 - l2**2) / (2 * l1 * r))
    beta = np.arctan2(y, x)
    theta1 = beta - alpha
    theta3 = np.pi - theta1 - theta2

    return theta1, theta2, theta3

# update the plot for each frame
def update(frame, arm_line, trace_line):
    theta1, theta2, theta3 = inverse_kinematics(1.5 * np.cos(frame) + 0, 1.5 * np.sin(frame) + 0)

    x, y = forward_kinematics(theta1, theta2, theta3)

    # Update the manipulator arm
    arm_line.set_xdata([0, l1 * np.cos(theta1), l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2), x])
    arm_line.set_ydata([0, l1 * np.sin(theta1), l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2), y])

    # Update the trace line
    trace_line.set_xdata(np.append(trace_line.get_xdata(), x))
    trace_line.set_ydata(np.append(trace_line.get_ydata(), y))

    return arm_line, trace_line

fig, ax = plt.subplots()
ax.set_xlim([-4, 4])
ax.set_ylim([-4, 4])

arm_line, = ax.plot([], [], 'o-', lw=2)
trace_line, = ax.plot([], [], 'r-', lw=1)

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('Inverse kinematics for 3R manipulator without Jacobian')

ani = FuncAnimation(fig, update, frames=np.linspace(0, 2 * np.pi, 100), fargs=(arm_line, trace_line), blit=True)

plt.show()
