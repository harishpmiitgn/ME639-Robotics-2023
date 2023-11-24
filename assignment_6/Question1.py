import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Link lengths, joint angles, time variable, trajectory parameters
link1_length = 1
link2_length = 1
link3_length = 1
joint_angles = np.array([-np.pi/2, np.pi/4, -np.pi/2])
t = 0
omega = 1.1  # speed
radius = 2
time_step = 0.1
num_points = 150

# Function to calculate the Jacobian matrix for the robot
def jacobian_matrix(joint_angles):
    joint_angle1, joint_angle2, joint_angle3 = joint_angles
    c1, s1 = np.cos(joint_angle1), np.sin(joint_angle1)
    c12, s12 = np.cos(joint_angle1 + joint_angle2), np.sin(joint_angle1 + joint_angle2)
    c123, s123 = np.cos(joint_angle1 + joint_angle2 + joint_angle3), np.sin(joint_angle1 + joint_angle2 + joint_angle3)

    J = np.array([
        [-link1_length * s1 - link2_length * s12 - link3_length * s123, -link2_length * s12 - link3_length * s123, -link3_length * s123],
        [link1_length * c1 + link2_length * c12 + link3_length * c123, link2_length * c12 + link3_length * c123, link3_length * c123]
    ])

    return J

# Function to calculate forward kinematics for the robot
def forward_kinematics(joint_angles):
    joint_angle1, joint_angle2, joint_angle3 = joint_angles

    x = np.sum([link1_length * np.cos(joint_angle1),
                link2_length * np.cos(joint_angle1 + joint_angle2),
                link3_length * np.cos(joint_angle1 + joint_angle2 + joint_angle3)])

    y = np.sum([link1_length * np.sin(joint_angle1),
                link2_length * np.sin(joint_angle1 + joint_angle2),
                link3_length * np.sin(joint_angle1 + joint_angle2 + joint_angle3)])

    return x, y

# Function to calculate inverse kinematics for the robot
def inverse_kinematics(x, y, phi):
    x2 = x - link3_length * np.cos(phi)
    y2 = y - link3_length * np.sin(phi)

    r_squared = x2**2 + y2**2
    atan_term = np.arctan2(y2, x2)

    cos_term = (r_squared - link1_length**2 - link2_length**2) / (2 * link1_length * link2_length)
    joint_angle2 = np.arccos(np.clip(cos_term, -1, 1))

    joint_angle1 = atan_term - np.arctan2(link2_length * np.sin(joint_angle2),
                                          link1_length + link2_length * np.cos(joint_angle2))
    
    joint_angle3 = np.deg2rad(90) - joint_angle1 - joint_angle2

    return joint_angle1, joint_angle2, joint_angle3

# Function to generate a circular trajectory
def generate_trajectory(t):
    x = radius * np.cos(omega * t)
    y = radius * np.sin(omega * t)
    return x, y

# Function to calculate velocity for a given time
def calculate_velocity(t):
    Vx = -omega * radius * np.sin(omega * t)
    Vy = omega * radius * np.cos(omega * t)
    return np.array([Vx, Vy])

# Function to display the robot in the animation
def display_robot(i):
    global joint_angles, t
    ax.clear()

    joint_angle1, joint_angle2, joint_angle3 = joint_angles
    end_effector_x, end_effector_y = forward_kinematics(joint_angles)
    end_effector_trace.append((end_effector_x, end_effector_y))
    t += time_step

    # Plot robot links
    link1_end = np.array([link1_length * np.cos(joint_angle1), link1_length * np.sin(joint_angle1)])
    link2_end = link1_end + np.array([link2_length * np.cos(joint_angle1 + joint_angle2), link2_length * np.sin(joint_angle1 + joint_angle2)])
    link3_end = np.array([end_effector_x, end_effector_y])
    links_x = np.array([0, link1_end[0], link2_end[0], link3_end[0]])
    links_y = np.array([0, link1_end[1], link2_end[1], link3_end[1]])
    ax.plot(links_x, links_y, 'b-')
    ax.plot(0,0, 'ro')

    # Plot joint positions and end effector
    joint_positions = np.array([link1_end, link2_end, link3_end])
    ax.plot(joint_positions[:, 0], joint_positions[:, 1], 'ro')
    ax.plot(end_effector_x, end_effector_y, 'ro')

    ax.set_xlim(-link1_length - link2_length - link3_length, link1_length + link2_length + link3_length)
    ax.set_ylim(-link1_length - link2_length - link3_length, link1_length + link2_length + link3_length)
    ax.set_title("3-Link Manipulator with Circular Trajectory")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)

    xd, yd = generate_trajectory(t)
    ax.plot(xd, yd)
    trajectory_trace.append((xd, yd))
    trajectory_x, trajectory_y = zip(*trajectory_trace)
    end_effector_x, end_effector_y = zip(*end_effector_trace)
    ax.plot(trajectory_x, trajectory_y)
    ax.plot(end_effector_x, end_effector_y, 'k:', linewidth=0.5)

    # Update joint angles based on velocity
    J = jacobian_matrix(joint_angles)
    x, y = forward_kinematics(joint_angles)
    xd, yd = generate_trajectory(t)
    error = np.array([x - xd, y - yd])
    joint_angles = joint_angles + time_step * np.linalg.lstsq(J, (calculate_velocity(t) - error), rcond=None)[0]

# Animation
fig, ax = plt.subplots()
end_effector_trace = []
trajectory_trace = []
ani = FuncAnimation(fig, display_robot, frames=range(800), interval=100, repeat=False)
plt.show()
