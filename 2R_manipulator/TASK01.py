import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 1.0  # link length
l2 = 1.5  # link length

def des_curve(t):
    x = 1.5*np.cos(t)
    y = 1.5*np.sin(t)
    return x, y


def kinematics(x, y):
    cos = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sin = np.sqrt(1 - cos**2)
    q2 = np.arctan2(sin, cos)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * sin, l1 + l2 * cos)
    return q1, q2


fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
link_A, = ax.plot([], [], 'r-', lw=2)
link_B, = ax.plot([], [], 'y-', lw=2)
endpoint, = ax.plot([], [], 'k+')

ts = 100
t_values = np.linspace(0, 2 * np.pi, ts)

def Visual(frame):
    t = t_values[frame]
    x_d, y_d = des_curve(t)
    q1, q2 = kinematics(x_d, y_d)
    
    x1 = l1 * np.cos(q1)
    y1 = l1 * np.sin(q1)
    
    x2 = x1 + l2 * np.cos(q1 + q2)
    y2 = y1 + l2 * np.sin(q1 + q2)
    
    link_A.set_data([0, x1], [0, y1])
    link_B.set_data([x1, x2], [y1, y2])
    endpoint.set_data(x2, y2)
    
    return link_A, link_B, endpoint   

ani = FuncAnimation(fig, Visual, frames=ts, blit=True)

plt.show()
