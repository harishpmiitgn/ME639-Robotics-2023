from math import sin, cos, acos, atan
import numpy as np
from numpy import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 2      # length of arm 1 (m)
l2 = 1      # length of arm 2 (m)

# let E (x,y) be end effector position

""" End effector range will be between two concentric circles with centres at origin and radius (l1 + l2) 
and radius (l1 - l2) """

""" Let the arbitarary trajectory of end effector be ellipse """
# Ellipse parameters
center = (2.5, 0)
width = 0.5
height = 0.25

# Generate points for the ellipse
t = linspace(0, 2*pi, 100)
x2 = center[0] + width * cos(t)
y2 = center[1] + height * sin(t)

# creating array for coordinates of joint O2
x1=linspace(0,0,100)
y1=linspace(0,0,100)

x0=[0]*100       # x-coordinate of joint O1
y0=[0]*100       # y-coordinate of joint O1

for i in range(100):
    x=x2[i]
    y=y2[i]
    theta =  acos( ((x**2) + (y**2) - (l1**2) - (l2**2)) / (2 * l1 * l2) )
    theta1 = atan(y / x) - atan( (l2 * sin(theta) ) / ( l1 + (l2 * cos(theta) )))
    x1[i] = (l1 * cos(theta1))        # x-coordinate of joint O2
    y1[i] = (l1 * sin(theta1))        # y-coordinate of joint O2 
    #theta2 = theta + theta1


#creating figure
fig, ax = plt.subplots()
ax.set_xlim(-1, 3.5)
ax.set_ylim(-1.5, 1)
ax.set_aspect('equal', adjustable='box')

# Enable grid lines
ax.grid(True)

# Initialize the plot elements
line1, = ax.plot([], [], 'b-', linewidth=2)  # Line for arm 1
line2, = ax.plot([], [], 'g-', linewidth=2)  # Line for arm 2
point_O1, = ax.plot([], [], 'ko', markersize=8)  # Joint O1
point_O2, = ax.plot([], [], 'ko', markersize=8)  # Joint O2
point_E, = ax.plot([], [], 'r*', markersize=8)   # End effector E
trace_line, = ax.plot([], [], 'k--', linewidth=1)  # Trace line

trace_x = []  # To store the traced x-coordinates
trace_y = []  # To store the traced y-coordinates

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    point_O1.set_data([], [])
    point_O2.set_data([], [])
    point_E.set_data([], [])
    trace_line.set_data([], [])
    return line1, line2, point_O1, point_O2, point_E, trace_line

def animate(i):
    line1.set_data([x0[i], x1[i]], [y0[i], y1[i]])
    line2.set_data([x1[i], x2[i]], [y1[i], y2[i]])
    point_O1.set_data(x0[i], y0[i])
    point_O2.set_data(x1[i], y1[i])
    point_E.set_data(x2[i], y2[i])
    
    trace_x.append(x2[i])
    trace_y.append(y2[i])
    trace_line.set_data(trace_x, trace_y)
    
    return line1, line2, point_O1, point_O2, point_E, trace_line

ani = FuncAnimation(fig, animate, frames=len(x1), init_func=init, blit=True)
plt.title("2R Manipulator End effector path tracing simulation.")
plt.show()
