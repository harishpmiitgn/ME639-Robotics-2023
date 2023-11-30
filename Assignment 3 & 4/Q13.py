import numpy as np
import math

def invkin_scara(T, a1, a2, d4):

    dx, dy, dz = T[:3, 3]
    R = T[:3, :3]

    r2 = (dx ** 2 + dy ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)

    # Checking workspace
    if r2 > 1 or r2 < -1:
        return None  # Position is not reachable

    theta2 = math.atan2((1 - r2 ** 2) ** 0.5, r2)
    theta1 = math.atan2(dy, dx) - math.atan2(a2 * math.sin(theta2), a1 + a2 * math.cos(theta2))
    alpha = math.atan2(R[0][1], R[0][0])
    theta4 = theta1 + theta2 - alpha

    d3 = dz + d4

    return theta1, theta2, d3, theta4
