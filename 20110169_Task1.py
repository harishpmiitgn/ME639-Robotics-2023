import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

# Robot parameters
l1 = 0.6
l2 = 0.3

# Control gains
kp = 10.0

# Desired arbitrary trajectory function
def desired_trajectory(t):
    x = 0.6 * np.cos(2 * t)
    y = 0.4 * np.sin(t)
    return x, y

# Dynamics equations
def dynamics(t, state):
    q1, q2, q1_dot, q2_dot = state
    x_desired, y_desired = desired_trajectory(t)
    
    # Inverse kinematics (assuming q1 is the base joint)
    q1_desired = np.arctan2(y_desired, x_desired)
    q2_desired = np.arccos((x_desired**2 + y_desired**2 - l1**2 - l2**2) / (2 * l1 * l2))
    
    # Control law
    q1_error = q1_desired - q1
    q2_error = q2_desired - q2
    tau1 = kp * q1_error
    tau2 = kp * q2_error
    
    # Dynamics
    q1_ddot = (tau1 - l2 * np.sin(q2) * tau2) / (l1**2 + l2**2 + 2 * l1 * l2 * np.cos(q2))
    q2_ddot = (tau2 - l1 * l2 * np.cos(q2) * tau1) / (l1**2 + l2**2 + 2 * l1 * l2 * np.cos(q2))
    
    return [q1_dot, q2_dot, q1_ddot, q2_ddot]

# Simulation parameters
t_span = (0.0, 10.0)
initial_state = [np.pi / 4, np.pi / 4, 0.0, 0.0]
sol = solve_ivp(dynamics, t_span, initial_state, t_eval=np.linspace(*t_span, 500))

# Forward kinematics
x_end = l1 * np.cos(sol.y[0]) + l2 * np.cos(sol.y[0] + sol.y[1])
y_end = l1 * np.sin(sol.y[0]) + l2 * np.sin(sol.y[0] + sol.y[1])

# Initialize the plot
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
line1, = ax.plot([], [], 'o-', lw=2, markersize=8, label='Arm 1')
line2, = ax.plot([], [], 'o-', lw=2, markersize=8, label='Arm 2')
desired_line, = ax.plot([], [], '--', color='gray', label='Desired Path')
trace_line, = ax.plot([], [], '-r', label='End Tip Path')

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    desired_line.set_data([], [])
    trace_line.set_data([], [])
    return line1, line2, desired_line, trace_line

def animate(i):
    arm1_x = [0, l1 * np.cos(sol.y[0, i])]
    arm1_y = [0, l1 * np.sin(sol.y[0, i])]
    
    arm2_x = [arm1_x[1], x_end[i]]
    arm2_y = [arm1_y[1], y_end[i]]
    
    desired_x, desired_y = desired_trajectory(sol.t[i])
    
    trace_x = x_end[:i+1]
    trace_y = y_end[:i+1]
    
    line1.set_data(arm1_x, arm1_y)
    line2.set_data(arm2_x, arm2_y)
    desired_line.set_data(desired_x, desired_y)
    trace_line.set_data(trace_x, trace_y)
    
    return line1, line2, desired_line, trace_line

ani = FuncAnimation(fig, animate, frames=len(sol.t), init_func=init, blit=True)

# Add legend
ax.legend()

plt.xlabel('X')
plt.ylabel('Y')
plt.title('2R Manipulator with Arbitrary End-Tip Trajectory')
plt.grid(True)
plt.show()
