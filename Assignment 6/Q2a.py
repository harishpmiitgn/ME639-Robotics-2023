import numpy as np

l1, l2, l3 = 0.25, 0.25, 0.25

def inverse_kinematics_puma(x, y, z, l1, l2, l3):

    # Assuming a simple geometric solution for PUMA robot
    theta1 = np.arctan2(y, x)

    # Solving for theta2 and theta3 using cosine law
    r = np.sqrt(x**2 + y**2)
    D = (r**2 + (z - l1)**2 - l2**2 - l3**2) / (2 * l2 * l3)
    theta3 = np.arctan2(np.sqrt(1 - D**2), D)

    phi = np.arctan2(z - l1, r)
    psi = np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
    theta2 = phi - psi

    return theta1, theta2, theta3

def forward_kinematics_puma(theta1, theta2, theta3, l1, l2, l3):

    x = (l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)) * np.cos(theta1)
    y = (l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)) * np.sin(theta1)
    z = l1 + l2*np.sin(theta2) + l3*np.sin(theta2 + theta3)

    return x, y, z

# Define the corner points of the manifold block
points = {
    "A": (0.45, 0.075, 0.1),
    "B": (0.45, -0.075, 0.1),
    "C": (0.25, -0.075, 0.1),
    "D": (0.25, 0.075, 0.1)
}

# Calculate the inverse kinematics for each point
ik_solutions = {}
for point, coords in points.items():
    ik_solutions[point] = inverse_kinematics_puma(*coords, l1, l2, l3)

# Calculate the forward kinematics to verify the positions
fk_positions = {}
for point, angles in ik_solutions.items():
    fk_positions[point] = forward_kinematics_puma(*angles, l1, l2, l3)

# Output the results
print(ik_solutions)
print(fk_positions)
