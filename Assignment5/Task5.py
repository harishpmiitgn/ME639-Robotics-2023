import numpy as np

def inverse_kinematics(x, y, z, roll, pitch, yaw):
    # Assuming a standard 6-DOF robotic arm with a spherical wrist

    # Constants (adjust these according to your robot's specifications)
    d1 = 1
    l2 = 1
    l3 = 1
    d6 = 1

    # Wrist center position (O3)
    wc_x = x - d6 * np.cos(yaw) * np.cos(pitch)
    wc_y = y - d6 * np.sin(yaw) * np.cos(pitch)
    wc_z = z + d6 * np.sin(pitch)

    # Inverse kinematics for the first three joints
    theta1 = np.arctan2(wc_y, wc_x)

    d = np.sqrt(wc_x**2 + wc_y**2 + (wc_z - d1)**2)
    a = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
    beta = np.arctan2(np.sqrt(1 - a**2), a)

    theta3 = np.pi - beta - np.arctan2(l3 * np.sin(beta), l2 + l3 * np.cos(beta))

    k1 = l2 + l3 * np.cos(beta)
    k2 = l3 * np.sin(beta)
    thetl2 = np.arctan2(wc_z - d1, np.sqrt(wc_x**2 + wc_y**2)) - np.arctan2(k2, k1)

    # Inverse kinematics for the last three joints of the wrist
    theta4 = roll - thetl2 - theta3

    # Ensure theta4 is within -pi to pi range
    theta4 = (theta4 + np.pi) % (2 * np.pi) - np.pi

    theta5 = pitch - thetl2 - theta3

    theta6 = yaw - theta1

    return theta4, theta5, theta6
