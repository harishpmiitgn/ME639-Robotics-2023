import numpy as np
import matplotlib.pyplot as plt
from math import *
from matplotlib.animation import FuncAnimation

l1 = 0.25
l2 = 0.25
l3 = 0.25


num_steps = 500
t= np.linspace(0,10,num_steps)
dt = 0.02

def generate_trajectory(points, steps):
    trajectory = []

    for i in range(len(points)):
        start_point = points[i]
        end_point = points[(i + 1) % len(points)]

        for t in np.linspace(0, 1, steps, endpoint=False):
            interpolated_point = (1 - t) * np.array(start_point) + t * np.array(end_point)
            trajectory.append(interpolated_point)

    return np.array(trajectory)


# Define square points
A = [0.40, 0.06, 0.1]
B = [0.40, 0.01, 0.1]
C = [0.35, 0.01, 0.1]
D = [0.35, 0.06, 0.1]
# Combine points in a cyclic order
points = [A, B, C, D, A]
# Generate trajectory with 100 steps
trajectory = generate_trajectory(points, steps=100)

# Initialize joint angles and end effector trajectory
q = np.zeros((num_steps, 3))
end_effector_trajectory = trajectory


# Configuration 1
# q[0][0]=  0.14888994760949725
# q[0][1] = -0.8851696682987094
# q[0][2] = 1.0600826270863721

# Configuration 2
q[0][0]=  0.148889
q[0][1] = 0.174917
q[0][2] = -1.06008

for i in range(1,num_steps):
    # Calculate desired end effector position on the circle
    
    q1 = q[i-1][0]
    q2 = q[i-1][1]
    q3 = q[i-1][2]
    J = np.array([[-l3*sin(q1)*cos(q2+q3)-l2*sin(q1)*cos(q2), -l3*cos(q1)*sin(q2+q3)-l2*cos(q1)*sin(q2), -l3*cos(q1)*sin(q2+q3) ],
                  [ l3*cos(q1)*cos(q2+q3)+l2*cos(q1)*cos(q2), -l3*sin(q1)*sin(q2+q3)-l2*sin(q1)*sin(q2), -l3*sin(q1)*sin(q2+q3) ],
                  [                  0                      ,        l3*cos(q2+q3)+l2*cos(q2)          ,     l3*cos(q2+q3)      ]])
    
    J_inv = np.linalg.inv(J)
    #print(J_inv.shape)
    
    # Calculate Xdot
    Xdot = (end_effector_trajectory[i,:]-end_effector_trajectory[i-1,:]) / dt

    # Update joint angles
    q[i,:] = q[i-1,:] + np.matmul(J_inv,Xdot) * dt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-0.5, 0.5])  # Set fixed limits for the x-axis
ax.set_ylim([-0.5, 0.5])  # Set fixed limits for the y-axis
ax.set_zlim([0, 0.5])     # Set fixed limits for the z-axis

# Define colors for the links
colors = ['r', 'g', 'b']

# Function to update the plot in each animation frame
def update(frame):
    x0, y0, z0 = 0, 0, 0

    x1 = 0
    y1 = 0
    z1 = l1

    x2 = l2 * np.cos(frame[0]) * np.cos(frame[1])
    y2 = l2 * np.sin(frame[0]) * np.cos(frame[1])
    z2 = l1 + l2 * np.sin(frame[1])

    x3 = l2 * np.cos(frame[0]) * np.cos(frame[1]) + l3 * np.cos(frame[0]) * np.cos(frame[1] + frame[2])
    y3 = l2 * np.sin(frame[0]) * np.cos(frame[1]) + l3 * np.sin(frame[0]) * np.cos(frame[1] + frame[2])
    z3 = l1 + l2 * np.sin(frame[1]) + l3 * np.sin(frame[1] + frame[2])

    ax.cla()  # Clear previous frame

    # Plot links with different colors
    ax.plot([x0, x1], [y0, y1], [z0, z1], color=colors[0], marker='o')
    ax.plot([x1, x2], [y1, y2], [z1, z2], color=colors[1], marker='o')
    ax.plot([x2, x3], [y2, y3], [z2, z3], color=colors[2], marker='o')

    # Plot red line representing the end effector
    ax.plot([x3], [y3], [z3], color='red', marker='o')

    # Plot the tracing path
    ax.plot(end_effector_trajectory[:i, 0], end_effector_trajectory[:i, 1], end_effector_trajectory[:i, 2], color='gray', linestyle='--')

# Create the animation
animation = FuncAnimation(fig, update, frames=q, interval=dt * 1000, repeat=False)
plt.show()