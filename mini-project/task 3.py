import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

L1 =5 #length of link 1
L2=3.5 #length of link 2

start_point = np.array([2,4])  # Starting position
end_point = np.array([2,-4])  # deflection position
trajectory1 = np.linspace(start_point, end_point, 15)
trajectory2 = np.linspace(end_point,start_point, 15)
trajectory=np.concatenate((trajectory1,trajectory2))

# Initialize plot
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
plt.grid()
ax.set_ylim(-10, 10)
line, = ax.plot([], [], 'r-', lw=2)

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

def inverse_kinematics(x, y):
    theta2 = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    theta1 = np.arctan2(y,x) - np.arctan2(L2 * np.sin(theta2),( L1 + L2 * np.cos(theta2)))
    return theta1, theta2


# Animate the manipulator following the trajectory
ani = FuncAnimation(fig, animate, frames=len(trajectory), init_func=init, blit=False, repeat=True)
plt.text(2,4,"upper extrimity",color='blue')
plt.text(2,-4,"Lower extrimity",color='blue')
plt.text(2,0,"Mean Position",color='green')
ax.set_title('2R Manipulator acting as spring')
plt.show()
    
