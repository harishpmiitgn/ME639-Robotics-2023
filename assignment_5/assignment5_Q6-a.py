import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Define robot parameters
mass_link1, mass_link2, mass_link3 = 1.0, 1.0, 1.0  # Masses of links
length_link2, length_link3 = 1.0, 1.0  # Link lengths
distance_com2, distance_com3 = 0.5, 0.5  # Center of mass distances
gravitational_acceleration = 9.81  # Gravitational acceleration

# Simulation parameters
simulation_time = 20.0
num_steps = 10000
time_step = simulation_time / num_steps
time_values = np.linspace(0, simulation_time, num_steps)

# Initialize state variables
joint_angles = np.zeros(3)  # Joint angles
joint_velocities = np.zeros(3)  # Joint velocities
motor_accelerations = np.zeros(3)  # Motor accelerations
desired_torque_motor = np.array([0.01, 0.02, 0.04])  # Small constant motor torques

# Motor dynamics parameters
motor_inertias = np.array([1, 1.5, 0.5])  # Motor inertias
motor_damping_coefficients = np.array([2, 3, 2.5])  # Motor damping coefficients
motor_resistances = np.array([1, 1.5, 0.5])  # Motor resistances
motor_inductances = np.array([0.01, 0.015, 0.005])  # Motor inductances
back_emf_constants = np.array([0.2, 0.25, 0.1])  # Motor back-emf constants
torque_constants = np.array([0.1, 0.3, 0.4])  # Motor torque constants

# Simulation loop
joint_angle_data = []
torque_motor_data = []

for step in range(num_steps):
    # Calculate joint accelerations using the equations of motion
    cos_theta2 = np.cos(joint_angles[1])
    sin_theta2 = np.sin(joint_angles[1])
    cos_theta3 = np.cos(joint_angles[2])
    sin_theta3 = np.sin(joint_angles[2])
    cos_theta23 = np.cos(joint_angles[1] + joint_angles[2])
    sin_theta23 = np.sin(joint_angles[1] + joint_angles[2])

    inertia_matrix = np.array([
        [0.5 * mass_link1 * distance_com2**2 + 0.25 * mass_link2 * length_link2**2 * cos_theta2**2 +
         0.25 * mass_link3 * length_link3**2 * cos_theta23**2 + mass_link3 * length_link2**2 * cos_theta2**2 +
         mass_link2 * length_link2 * length_link3 * cos_theta2 * cos_theta23, 0, 0],
        [0, 0.25 * mass_link2 * length_link2**2 + mass_link3 * (length_link2**2 + distance_com3**2 + length_link2 * length_link3 * cos_theta3), 0],
        [0, 0, 1/3 * mass_link3 * length_link3**2]
    ])

    coriolis_matrix = np.array([
        [0, -0.25 * mass_link2 * length_link2**2 * sin_theta2 - mass_link3 * length_link2**2 * sin_theta2 -
         mass_link3 * length_link2 * length_link3 * sin_theta23 - mass_link3 * distance_com2**2 * sin_theta23,
         -mass_link3 * length_link2 * length_link3 * cos_theta2 * sin_theta23 - mass_link3 * distance_com3**2 * sin_theta23],
        [0, 0, -mass_link2 * length_link2 * length_link3 * sin_theta3],
        [0, 0, 0]
    ])

    gravitational_vector = np.array([
        distance_com2 * mass_link2 * gravitational_acceleration * cos_theta2 + length_link2 * mass_link3 * gravitational_acceleration * cos_theta2 +
        distance_com3 * mass_link2 * gravitational_acceleration * cos_theta23,
        0,
        mass_link3 * gravitational_acceleration * distance_com3 * cos_theta23
    ])

    torque_motor = desired_torque_motor - motor_damping_coefficients * joint_velocities - torque_constants * (1 / motor_resistances) * (back_emf_constants * joint_velocities)
    motor_accelerations = np.linalg.solve(np.diag(motor_inertias), torque_motor - np.dot(coriolis_matrix, joint_velocities) - gravitational_vector)

    joint_velocities = joint_velocities + motor_accelerations * time_step
    joint_angles = joint_angles + joint_velocities * time_step

    # Store data for analysis
    joint_angle_data.append(joint_angles.copy())
    torque_motor_data.append(torque_motor.copy())

# Convert data lists to numpy arrays for easier analysis or plotting
joint_angle_data = np.array(joint_angle_data)
torque_motor_data = np.array(torque_motor_data)

# Plot joint angles and torques
time_steps = np.arange(num_steps) * time_step
plt.figure(figsize=(12, 6))
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(time_steps, joint_angle_data[:, i], label=f'Joint {i+1} Angle')
    plt.ylabel(f'Joint {i+1} Angle (rad)')
    plt.grid()
plt.xlabel('Time (s)')
plt.show()

plt.figure(figsize=(12, 6))
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(time_steps, torque_motor_data[:, i], label=f'Joint {i+1} Torque')
    plt.ylabel(f'Joint {i+1} Torque (Nm)')
    plt.grid()
plt.xlabel('Time (s)')
plt.show()
