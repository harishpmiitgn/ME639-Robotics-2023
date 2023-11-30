import numpy as np
import matplotlib.pyplot as plt

l1, l2, l3 = 0.25, 0.25, 0.25

def inv_kin(x, y, z, l1, l2, l3):

    theta1 = np.arctan2(y, x)

    r = np.sqrt(x**2 + y**2)
    d = (r ** 2 + (z - l1) ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)

    theta3 = np.arctan2(np.sqrt(1 - d ** 2), d)
    phi = np.arctan2(z - l1, r)
    psi = np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
    theta2 = phi - psi

    return theta1, theta2, theta3

def for_kin(theta1, theta2, theta3, l1, l2, l3):

    x = (l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)) * np.cos(theta1)
    y = (l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)) * np.sin(theta1)
    z = l1 + l2*np.sin(theta2) + l3*np.sin(theta2 + theta3)

    return x, y, z

def trajectory(s, e, n):

    return np.linspace(s, e, n)

A = np.array([0.40, 0.06, 0.1])
B = np.array([0.40, 0.01, 0.1])
C = np.array([0.35, 0.01, 0.1])
D = np.array([0.35, 0.06, 0.1])


n_side = 20

# Generate the trajectory
trajectory = np.concatenate([
    trajectory(A, B, n_side),
    trajectory(B, C, n_side),
    trajectory(C, D, n_side),
    trajectory(D, A, n_side)
])

# inverse kinematics
joint_angles = np.array([inv_kin(x, y, z, l1, l2, l3) for x, y, z in trajectory])
# forward kinematics
cartesian_coordinates = np.array([for_kin(theta1, theta2, theta3, l1, l2, l3) for theta1, theta2, theta3 in joint_angles])

time = np.linspace(0, len(trajectory), len(trajectory))


plt.figure(figsize=(15, 10))

# Plotting joint angles
plt.subplot(2, 1, 1)
plt.plot(time, joint_angles[:, 0], label='Theta1')
plt.plot(time, joint_angles[:, 1], label='Theta2')
plt.plot(time, joint_angles[:, 2], label='Theta3')
plt.xlabel('t')
plt.ylabel('Joint Angles (rad)')

plt.legend()
plt.grid()

# Plotting Cartesian coordinates
plt.subplot(2, 1, 2)
plt.plot(time, cartesian_coordinates[:, 0], label='X')
plt.plot(time, cartesian_coordinates[:, 1], label='Y')
plt.plot(time, cartesian_coordinates[:, 2], label='Z')
plt.xlabel('t')
plt.ylabel('Cartesian Coordinates (m)')
plt.title('Cartesian Coordinates over Time')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
