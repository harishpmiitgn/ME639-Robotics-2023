
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

 # link length
l1 = 1.0  
l2 = 2.0  
l3 = 3.0  
phie = 30

  # Trajectory equation
def des_curve(t):
    x = 1.5*np.cos(t)
    y = 1.5*np.sin(t)
    return x, y

def inverse_kine(x, y):
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    s2 = np.sqrt(1 - c2**2)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * s2, l1 + l2 * c2)
    q2 = np.arctan2(s2, c2)
    q3 = phie - (q2+q1)
    return q1, q2, q3

timesteps = 100
t_vals = np.linspace(0, 2 * np.pi, timesteps)

#Animation
fig, ax = plt.subplots()
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)

link_1, = ax.plot([], [], 'b-', lw=2)
link_2, = ax.plot([], [], 'g-', lw=2)
link_3, = ax.plot([], [], 'r-', lw=2)
endpoint, = ax.plot([], [], 'ro')

def animation(frame):
    t = t_vals[frame]
    x_d, y_d = des_curve(t)
    q1, q2, q3 = inverse_kine(x_d, y_d)
    x1 = l1 * np.cos(q1)
    y1 = l1 * np.sin(q1)
    x2 = x1 + l2 * np.cos(q1 + q2)
    y2 = y1 + l2 * np.sin(q1 + q2)
    x3 = x2 + l3 * np.cos(phie)
    y3 = y2 + l3 * np.sin(phie)
    link_1.set_data([0, x1], [0, y1])
    link_2.set_data([x1, x2], [y1, y2])
    link_3.set_data([x2, x3], [y2, y3])
    endpoint.set_data(x3, y3)
    return link_1, link_2, link_3, endpoint   

ani = FuncAnimation(fig, animation, frames=timesteps, blit=True)
plt.show()