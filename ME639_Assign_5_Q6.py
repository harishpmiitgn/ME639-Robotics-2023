import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# Robot Dynamics Parameters
masses = np.array([1.0, 1.0, 1.0])
lengths = np.array([1.0, 1.0, 1.0])
inertias = np.array([0.1, 0.1, 0.1])
g = 9.8

# Control Gains
kp, kd, ki = 10.0, 1.0, 0.1

# Time Step
time_step = 0.01

# Simulated Time Steps
num_steps = 1000

# Small Initial Conditions
initial_state = np.array([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

# Constant Torques
constant_torques = np.array([0.001, 0.001, 0.001])

# Robot Dynamics Function
def robot_dynamics(state, control_input):
    q1_ddot = control_input[0] / inertias[0]
    q2_ddot = control_input[1] / inertias[1]
    q3_ddot = control_input[2] / inertias[2]

    q1_dot_new = state[3] + q1_ddot * time_step
    q2_dot_new = state[4] + q2_ddot * time_step
    q3_dot_new = state[5] + q3_ddot * time_step
    q1_new = state[0] + q1_dot_new * time_step
    q2_new = state[1] + q2_dot_new * time_step
    q3_new = state[2] + q3_dot_new * time_step

    return np.array([q1_new, q2_new, q3_new, q1_dot_new, q2_dot_new, q3_dot_new])

# Independent Joint Control Functions

def simple_control(state, desired_state):
    error = desired_state - state[:3]
    control_input = kp * error - kd * state[3:6]
    return control_input

def sophisticated_control(state, desired_state):
    error = desired_state - state[:3]
    control_input = kp * error - kd * state[3:6] + ki * np.sum(error)
    return control_input

def feedforward_control(state, desired_state, desired_traj):
    error = desired_state - state[:3]
    control_input = kp * error - kd * state[3:6] + desired_traj
    return control_input

def computed_torque_control(state, desired_state):
    q1, q2, q3, q1_dot, q2_dot, q3_dot = state
    q1_des, q2_des, q3_des = desired_state

    error = np.array([q1_des - q1, q2_des - q2, q3_des - q3])
    error_dot = np.array([-q1_dot, -q2_dot, -q3_dot])

    M = np.array([[inertias[0], 0, 0],
                  [0, inertias[1], 0],
                  [0, 0, inertias[2]]])

    C = np.zeros(3)

    G = np.array([0, masses[1] * g * lengths[1] * np.cos(q2), masses[2] * g])

    control_input = np.linalg.inv(M) @ (kp * error + kd * error_dot + G + C)
    return control_input

# Desired Trajectory Function
def desired_trajectory(time):
    q1_desired = np.sin(time)
    q2_desired = np.cos(time)
    q3_desired = np.sin(time) / 2
    return np.array([q1_desired, q2_desired, q3_desired])

# Simulate Robot Function
def simulate_robot(control_method, num_steps, time_step, disturbance_std=0.01):
    state = initial_state.copy()

    states = []

    for step in range(num_steps):
        time = step * time_step
        desired_state = desired_trajectory(time) + np.random.normal(0, disturbance_std, 3)

        if control_method == feedforward_control:
            control_input = control_method(state, desired_state, desired_trajectory(time))
        else:
            control_input = control_method(state, desired_state)

        state = robot_dynamics(state, control_input)
        states.append(state)

    return np.array(states)

# Simulation with Constant Torques
state_history_constant_torques = np.zeros((num_steps, len(initial_state)))
state_history_constant_torques[0, :] = initial_state

for step in range(1, num_steps):
    previous_state = state_history_constant_torques[step - 1, :]
    new_state = robot_dynamics(previous_state, constant_torques)
    state_history_constant_torques[step, :] = new_state

# Simulation and Plotting for Different Control Methods
simple_states = simulate_robot(simple_control, num_steps, time_step)
sophisticated_states = simulate_robot(sophisticated_control, num_steps, time_step)
feedforward_states = simulate_robot(feedforward_control, num_steps, time_step)
computed_torque_states = simulate_robot(computed_torque_control, num_steps, time_step)

# Plotting Joint Positions Over Time with Small Constant Torques
plt.figure(figsize=(14, 10))
plt.plot(time, state_history_constant_torques[:, 0], label='Joint 1 Position - Constant Torques', linestyle='-', color='blue')
plt.plot(time, state_history_constant_torques[:, 1], label='Joint 2 Position - Constant Torques', linestyle='--', color='orange')
plt.plot(time, state_history_constant_torques[:, 2], label='Joint 3 Position - Constant Torques', linestyle=':', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.title('Joint Positions Over Time with Small Constant Torques')
plt.legend()
plt.grid(True)
plt.show()

# Plotting Individual Control Methods
plt.figure(figsize=(14, 10))
gs = gridspec.GridSpec(2, 2, height_ratios=[1, 1])

plt.subplot(gs[0])
plt.plot(time, simple_states[:, 0], label='Simple Control - Joint 1', linestyle='-', color='blue')
plt.plot(time, simple_states[:, 1], label='Simple Control - Joint 2', linestyle='--', color='orange')
plt.plot(time, simple_states[:, 2], label='Simple Control - Joint 3', linestyle=':', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position')
plt.title('Simple Control')
plt.legend()
plt.grid(True)

plt.subplot(gs[1])
plt.plot(time, sophisticated_states[:, 0], label='Sophisticated Control - Joint 1', linestyle='-', color='blue')
plt.plot(time, sophisticated_states[:, 1], label='Sophisticated Control - Joint 2', linestyle='--', color='orange')
plt.plot(time, sophisticated_states[:, 2], label='Sophisticated Control - Joint 3', linestyle=':', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position')
plt.title('Sophisticated Control')
plt.legend()
plt.grid(True)

plt.subplot(gs[2])
plt.plot(time, feedforward_states[:, 0], label='Feedforward Control - Joint 1', linestyle='-', color='blue')
plt.plot(time, feedforward_states[:, 1], label='Feedforward Control - Joint 2', linestyle='--', color='orange')
plt.plot(time, feedforward_states[:, 2], label='Feedforward Control - Joint 3', linestyle=':', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position')
plt.title('Feedforward Control')
plt.legend()
plt.grid(True)

plt.subplot(gs[3])
plt.plot(time, computed_torque_states[:, 0], label='Computed Torque Control - Joint 1', linestyle='-', color='blue')
plt.plot(time, computed_torque_states[:, 1], label='Computed Torque Control - Joint 2', linestyle='--', color='orange')
plt.plot(time, computed_torque_states[:, 2], label='Computed Torque Control - Joint 3', linestyle=':', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position')
plt.title('Computed Torque Control')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Combined Plot for Control Comparison
plt.figure(figsize=(14, 10))

plt.plot(time, simple_states[:, 0], label='Simple Control - Joint 1', linestyle='-', marker='o', markevery=100, color='blue')
plt.plot(time, sophisticated_states[:, 0], label='Sophisticated Control - Joint 1', linestyle='--', marker='^', markevery=100, color='orange')
plt.plot(time, feedforward_states[:, 0], label='Feedforward Control - Joint 1', linestyle='-.', marker='s', markevery=100, color='green')
plt.plot(time, computed_torque_states[:, 0], label='Computed Torque Control - Joint 1', linestyle=':', marker='*', markevery=100, color='purple')

plt.xlabel('Time (s)')
plt.ylabel('Joint Position')
plt.title('Joint Control Comparison - Combined Plot')
plt.legend()
plt.grid(True)
plt.show()
