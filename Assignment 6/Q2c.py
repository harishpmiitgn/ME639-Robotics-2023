import numpy as np
import matplotlib.pyplot as plt

dt = 0.01
total_t = 5.0
t_steps = int(total_t / dt)

# PID Controller
Kp = 10  # Proportional gain
Ki = 0  # Integral gain
Kd = 20  # Derivative gain

# Robot dynamics parameters
inertia = 1.0
damping = 0.1

d_trajectory = np.sin(np.linspace(0, 2 * np.pi, t_steps)) # Desired trajectory

angle = np.zeros(t_steps)  # Actual angle of the joint
velocity = np.zeros(t_steps)  # Velocity of the joint
torque = np.zeros(t_steps)  # Control input (torque)
e = np.zeros(t_steps)  # Error

for t in range(1, t_steps):
    # Calculate current error
    e[t] = d_trajectory[t] - angle[t - 1]

    # PID Control (Feedforward + Feedback)
    torque[t] = (d_trajectory[t] * Kp) + (Kp * e[t]) + (Kd * (e[t] - e[t - 1]) / dt)

    # Small random disturbance -> torque
    disturbance = np.random.normal(0, 0.1)
    torque[t] += disturbance

    angular_acceleration = (torque[t] - damping * velocity[t-1]) / inertia
    velocity[t] = velocity[t-1] + angular_acceleration * dt
    angle[t] = angle[t-1] + velocity[t] * dt

# Plotting the results
plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.plot(np.linspace(0, total_t, t_steps), d_trajectory, label="Desired Trajectory")
plt.plot(np.linspace(0, total_t, t_steps), angle, label="Actual Angle", linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Joint Angle Over Time")
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(np.linspace(0, total_t, t_steps), torque, label="Applied Torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Control Input (Torque) Over Time")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
