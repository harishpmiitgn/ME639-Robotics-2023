import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Manipulator parameters
L1 = 1.0  # Length of the first link
L2 = 1.0  # Length of the second link

# Path equation
def desired_curve(t):
    x = np.cos(t)
    y = np.sin(t)
    return x, y

def inverse_kinematics(x, y):
    c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    s2 = np.sqrt(1 - c2**2)
    q1 = np.arctan2(y, x) - np.arctan2(L2 * s2, L1 + L2 * c2)
    q2 = np.arctan2(s2, c2)
    return q1, q2

timesteps = 100
t_vals = np.linspace(0, 2 * np.pi, timesteps)

# Animation
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

link1, = ax.plot([], [], 'b-', lw=2)
link2, = ax.plot([], [], 'g-', lw=2)
endpoint, = ax.plot([], [], 'ro')
trace, = ax.plot([], [], 'r--', lw=1)  # Line for tracing the end effector path

trace_x = []  # Store x coordinates of end effector trace
trace_y = []  # Store y coordinates of end effector trace

def func_animation(frame):
    t = t_vals[frame]
    x_d, y_d = desired_curve(t)
    q1, q2 = inverse_kinematics(x_d, y_d)
    
    # Convert radians to degrees for angle comparison
    q1_deg = np.degrees(q1)
    q2_deg = np.degrees(q2)
    
    # Check if q1 and q2 are within the desired angle range
    if 35 <= q1_deg <= 145 and 35 <= q2_deg <= 145:
        x1 = L1 * np.cos(q1)
        y1 = L1 * np.sin(q1)
        x2 = x1 + L2 * np.cos(q1 + q2)
        y2 = y1 + L2 * np.sin(q1 + q2)
        
        link1.set_data([0, x1], [0, y1])
        link2.set_data([x1, x2], [y1, y2])
        endpoint.set_data(x2, y2)
        
        trace_x.append(x2)  # Add x coordinate to trace list
        trace_y.append(y2)  # Add y coordinate to trace list
        trace.set_data(trace_x, trace_y)  # Update trace plot
    
    return link1, link2, endpoint, trace

ani = FuncAnimation(fig, func_animation, frames=timesteps, blit=True)
plt.show()