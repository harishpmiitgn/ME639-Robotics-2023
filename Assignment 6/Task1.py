import numpy as np
import matplotlib.pyplot as plt
from math import *
from matplotlib.animation import FuncAnimation

num_steps = 100
t= np.linspace(0,10,num_steps)
dt = 0.1

# Initialize joint angles and end effector trajectory
q = np.zeros((num_steps, 3))
end_effector_trajectory = np.zeros((num_steps, 2))

end_effector_trajectory[0][0] = 1.5
end_effector_trajectory[0][1] = 0

q[0][0]=  0
q[0][1] = 1.318
q[0][2] = -2.636
for i in range(1,num_steps):
    # Calculate desired end effector position on the circle
    theta = 2 * np.pi * i / num_steps
    end_effector_position = np.array([np.cos(theta), np.sin(theta)]) * 1.5
    end_effector_trajectory[i, :] = end_effector_position
    q1 = q[i-1][0]
    q2 = q[i-1][1]
    q3 = q[i-1][2]
    J = np.array([[-sin(q1)-sin(q1+q2)-sin(q1+q2+q3), -sin(q1+q2)-sin(q1+q2+q3), -sin(q1+q2+q3) ],
                  [ cos(q1)+cos(q1+q2)+cos(q1+q2+q3),  cos(q1+q2)+cos(q1+q2+q3),  cos(q1+q2+q3) ]])
    
    J_T = np.transpose(J)
    J1 = np.matmul(J,J_T)
    J1_inv = np.linalg.inv(J1)
    J_inv = np.matmul(J_T,J1_inv)
    #print(J_inv.shape)
    
    # Calculate Xdot
    Xdot = (end_effector_trajectory[i,:]-end_effector_trajectory[i-1,:]) / dt

    # Update joint angles
    q[i,:] = q[i-1,:] + np.matmul(J_inv,Xdot) * dt

fig, ax = plt.subplots()
line, = ax.plot([], [], marker='o')
trajectory_line, = ax.plot([], [], color='red', linestyle='--', linewidth=1, animated=True)

def init():
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    return line,

def update(frame):
    x0, y0 = 0, 0
    x1 = np.cos(frame[0])
    y1 = np.sin(frame[0])

    x2 = x1 + np.cos(frame[0] + frame[1])
    y2 = y1 + np.sin(frame[0] + frame[1])

    x3 = x2 + np.cos(frame[0] + frame[1] + frame[2])
    y3 = y2 + np.sin(frame[0] + frame[1] + frame[2])

    line.set_data([x0, x1, x2, x3], [y0, y1, y2, y3])
    # Update the trajectory line
    trajectory_line.set_xdata(np.append(trajectory_line.get_xdata(), x3))
    trajectory_line.set_ydata(np.append(trajectory_line.get_ydata(), y3))

    # Adjust axes limits dynamically
    ax.relim()
    ax.autoscale_view()

    return line, trajectory_line

# Create the animation
animation = FuncAnimation(fig, update, frames=q, init_func=init, blit=True)

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('3R Planar Manipulator Animation')
plt.grid(True)
plt.show()

