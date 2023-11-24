import numpy as np
import matplotlib.pyplot as plt

# Constants for robot dimensions and parameters
LINK_LENGTH_1 = 0.75
LINK_LENGTH_2 = 0.5
LINK_LENGTH_3 = 0.25
MASS_1, MASS_2, MASS_3 = 0.6, 0.8, 1
INERTIA_1, INERTIA_2, INERTIA_3 = 0.07, 0.09, 0.11
# Points A, B, C, D
points = np.array([
    [0.4, 0.06, 0.1],
    [0.4, 0.01, 0.1],
    [0.35, 0.01, 0.1],
    [0.35, 0.06, 0.1]
    ])

# Function to build a Denavit-Hartenberg transformation matrix
def build_dh_matrix(d_param, theta_param, alpha_param, a_param):
    c_theta, s_theta = np.cos(theta_param), np.sin(theta_param)
    c_alpha, s_alpha = np.cos(alpha_param), np.sin(alpha_param)

    dh = np.array([
        [c_theta, -s_theta * c_alpha, s_theta * s_alpha, a_param * c_theta],
        [s_theta, c_theta * c_alpha, -c_theta * s_alpha, a_param * s_theta],
        [0, s_alpha, c_alpha, d_param],
        [0, 0, 0, 1]
    ])
    return dh

# Function for SCARA robot inverse kinematics
def scara_inverse_kinematics(x_pos, y_pos, z_pos):
    theta_2 = np.arccos((x_pos**2 + y_pos**2 - LINK_LENGTH_1**2 - LINK_LENGTH_2**2) / (2 * LINK_LENGTH_1 * LINK_LENGTH_2))
    theta_1 = np.arctan2(y_pos, x_pos) - np.arctan2(LINK_LENGTH_2 * np.sin(theta_2), LINK_LENGTH_1 + LINK_LENGTH_2 * np.cos(theta_2))
    theta_1 = (theta_1 + np.pi) % (2 * np.pi) - np.pi
    d_3 = -z_pos
    return np.array([theta_1, theta_2, d_3])

# Function for SCARA robot forward kinematics
def forward_kinematics(q_param):
    theta_1, theta_2, d_3 = q_param
    T_03 = build_dh_matrix(0, theta_1, 0, LINK_LENGTH_1) @ build_dh_matrix(0, theta_2, np.pi, LINK_LENGTH_2) @ build_dh_matrix(d_3, 0, 0, 0)
    return T_03[:3, 3]

# Function to generate linear trajectory between two points
def genlinear_trajectory(initial, final, t_param, timetaken):
    return initial + (final - initial) * t_param / timetaken

# Trajectory variables
theta_1_traj, theta_2_traj, d_3_traj = [], [], []
x_pos_traj, y_pos_traj, z_pos_traj = [], [], []

# Generate time array for plotting
timetaken = 100
t_smol = np.linspace(0, timetaken, timetaken)
tnet = np.linspace(0, 4 * timetaken, 4 * timetaken)

# Generate trajectories for each point transition
for i in range(4):
    for t in t_smol:
        # Generate linear trajectory for end-effector position
        x_pos, y_pos, z_pos = genlinear_trajectory(points[i, :], points[(i + 1) % 4, :], t, timetaken)
        x_pos_traj.append(x_pos)
        y_pos_traj.append(y_pos)
        z_pos_traj.append(z_pos)

        # Calculate inverse kinematics for joint angles
        q = scara_inverse_kinematics(x_pos, y_pos, z_pos)
        theta_1_traj.append(q[0])
        theta_2_traj.append(q[1])
        d_3_traj.append(q[2])



# Plotting
fig, axs = plt.subplots(2, 3, figsize=(15, 15), sharex=True)

axs[0, 0].plot(tnet, x_pos_traj, label='x_pos')
axs[0, 1].plot(tnet, y_pos_traj, label='y_pos')
axs[0, 2].plot(tnet, z_pos_traj, label='z_pos')
axs[1, 0].plot(tnet, np.rad2deg(theta_1_traj), label='theta_1')
axs[1, 1].plot(tnet, np.rad2deg(theta_2_traj), label='theta_2')
axs[1, 2].plot(tnet, d_3_traj, label='d_3')

# Set labels and title
axs[1, 0].set_xlabel('Time')
axs[1, 1].set_xlabel('Time')
axs[1, 2].set_xlabel('Time')
fig.suptitle('Plots for SCARA')

# Display legends
for ax_row in axs:
    for ax in ax_row:
        ax.legend()

# Show the plot
plt.show()
