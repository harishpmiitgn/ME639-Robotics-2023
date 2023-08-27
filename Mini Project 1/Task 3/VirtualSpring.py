import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Workspace boundaries
workspace_x_min = -3.5
workspace_x_max = 3.5
workspace_y_min = -3.5
workspace_y_max = 3.5

# Arm lengths (constant)
L1 = 2.0
L2 = 1.0

# Initial configuration
theta1 = np.pi/6
theta2 = np.pi/3

x0 = L1 * np.cos(theta1) + L2 * np.cos(theta2)
y0 = L1 * np.sin(theta1) + L2 * np.sin(theta2)

# store theta1 and theta2 values throughout the motion in list
dragging_thetas = []  # Declare dragging_thetas as a global variable

def forward_kinematics(theta1, theta2):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta2)
    return x, y

def inverse_kinematics(x, y):
    D = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    theta = np.arccos(D)
    theta1 = np.arctan(y/x) - np.arctan((L2 * np.sin(theta))/(L1 + L2 * np.cos(theta)))
    theta2 = theta1 + theta
    return theta1, theta2

def animate_frames(frame):
    global theta1, theta2
    if frame < len(dragging_thetas):
        theta1, theta2 = dragging_thetas[frame]
    else:
        theta1, theta2 = initial_theta1, initial_theta2
    update_plot()

def update_plot():
    ax.clear()
    ax.set_title('2R manipulator end tip acting like virtual spring')  # Set the title
    ax.plot(x0, y0, 'mo', label='Starting Point') # Plot starting point
    ax.plot([0, L1 * np.cos(theta1)], [0, L1 * np.sin(theta1)], 'b-o')  # First link
    ax.plot([L1 * np.cos(theta1), L1 * np.cos(theta1) + L2 * np.cos(theta2)], [L1 * np.sin(theta1), L1 * np.sin(theta1) + L2 * np.sin(theta2)], 'g-o')  # Second link
    ax.plot(L1 * np.cos(theta1), L1 * np.sin(theta1), 'go')  # Joint
    ax.plot(L1 * np.cos(theta1) + L2 * np.cos(theta2), L1 * np.sin(theta1) + L2 * np.sin(theta2), 'ro')  # End effector
    
    ax.set_xlim(workspace_x_min, workspace_x_max)
    ax.set_ylim(workspace_y_min, workspace_y_max)
    ax.legend()
    plt.draw()

fig, ax = plt.subplots()

update_plot()

is_dragging = False
initial_theta1, initial_theta2 = theta1, theta2

def on_click(event):
    global is_dragging
    if event.button == 1:  # Left mouse button 
        is_dragging = True
        dragging_thetas.clear()  # Clear the recorded angles

def on_release(event):
    global is_dragging
    if event.button == 1:  # Left mouse button
        is_dragging = False
        animate()
        dragging_thetas.reverse()  # Reverse the recorded angles
        update_plot()

def on_motion(event):
    global theta1, theta2
    if is_dragging:
        new_x = np.clip(event.xdata, workspace_x_min, workspace_x_max)
        new_y = np.clip(event.ydata, workspace_y_min, workspace_y_max)
        theta1, theta2 = inverse_kinematics(new_x, new_y)
        dragging_thetas.append((theta1, theta2))
        update_plot()

def animate():
    global anim
    anim = FuncAnimation(fig, animate_frames, frames=len(dragging_thetas)+1, interval=200, repeat=False)

fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('motion_notify_event', on_motion)

plt.show()
