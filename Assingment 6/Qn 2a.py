import numpy as np

l1 = 0.25
l2 = 0.25
l3 = 0.25

def calculate_end_effector_position(q1, q2, q3):
    A1 = np.array([
                    [np.cos(q1), -np.sin(q1), 0, 0],
                    [np.sin(q1), np.cos(q1), 0, 0],
                    [0, 1, 0, l1],
                    [0, 0, 0, 1]
    ])
    A2 = np.array([
                    [np.cos(q2), -np.sin(q2), 0, 0],
                    [0, 0, -1, 0],
                    [np.sin(q2), np.cos(q2), 0, 0],
                    [0, 0, 0, 1]
    ])
    A3 = np.array([
                    [np.cos(q3), -np.sin(q3), 0, 0],
                    [0, 0, 1, l2],
                    [-np.sin(q3), -np.cos(q3), 0, 0],
                    [0, 0, 0, 1]
    ])
    A4 = np.array([
                    [1, 0, 0, l3],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
    ])

    T = np.linalg.multi_dot([A1, A2, A3, A4])
    P = T[:3, 3]

    return P

# Inverse Kinematics
def puma_inv(x, y, z):
    q1 = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)
    D = (r**2 + (z - l1)**2 - l2**2 - l3**2) / (2 * l2 * l3)
    q2 = np.arctan2(np.sqrt(1 - D**2), D)
    gamma = np.arctan2(z - l1, r)
    beta = np.arctan2(l3 * np.sin(q2), l2 + l3 * np.cos(q2))
    q3 = np.pi/2 - (gamma - beta)
    return q1, q2, q3

# calculate joint angles and position
def calculate_joint_angles_and_position(point):
    x, y, z = point
    joint_angles = puma_inv(x, y, z)
    position = calculate_end_effector_position(*joint_angles)
    return joint_angles, position

# Corner points
points = {
           'A': [0.45, 0.075, 0.1],
           'B': [0.45, -0.075, 0.1],
           'C': [0.25, -0.075, 0.1],
           'D': [0.25, 0.075, 0.1]
}

results = {point_name: calculate_joint_angles_and_position(point) for point_name, point in points.items()}

for point_name, (joint_angles, position) in results.items():
    joint_angles = [round(angle, 4) for angle in joint_angles]
    position = [round(coord, 4) for coord in position]
    print(f"Point {point_name}:")
    print(f"Joint Angles: {joint_angles}")
    print(f"End Effector Position: {position}\n")
