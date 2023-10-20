import numpy as np
import math

def scara_inverse_kinematics(T, a1, a2, d4):

    # Extract position and rotation from the transformation matrix
    dx, dy, dz = T[:3, 3]
    R = T[:3, :3]

    # Compute r^2 using equation
    r2 = (dx ** 2 + dy ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)

    # Check if the position is reachable
    if r2 > 1 or r2 < -1:
        return None  # Position is not reachable

    # Compute theta2 using equation
    theta2 = math.atan2((1 - r2 ** 2) ** 0.5, r2)

    # Compute theta1 using equation
    theta1 = math.atan2(dy, dx) - math.atan2(a2 * math.sin(theta2), a1 + a2 * math.cos(theta2))

    # Compute theta4 using equation
    alpha = math.atan2(R[0][1], R[0][0])
    theta4 = theta1 + theta2 - alpha

    # Compute d3
    d3 = dz + d4

    return theta1, theta2, d3, theta4


# Test with code in Q4
