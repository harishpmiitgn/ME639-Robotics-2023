import numpy as np
from math import atan2, sqrt

def inverse_spherical_wrist(u):

    u13, u23, u31, u32, u33, u11, u21 = U[0, 2], U[1, 2], U[2, 0], U[2, 1], U[2, 2], U[0, 0], U[1, 0]

    c1 = np.arctan2(u23, u33)
    s1 = np.arctan2(-u13, np.sqrt(r23 ** 2 + r33 ** 2))

    c5 = np.cos(s1)
    if abs(c5) > 1e-6:
        s5 = r33 / c5
        theta5 = np.arctan2(s1 * r13 - c1 * r23, c1 * r13 + s1 * r23 - c5)

        theta4 = np.arctan2(c1 * r23 - s1 * r13, -c1 * r33 + s1 * r23)

        theta6 = np.arctan2(-s1 * r12 + c1 * r22, s1 * r11 - c1 * r21)

        return theta4, theta5, theta6
    else:

        theta4 = np.arctan2(r21, r11)
        theta6 = np.arctan2(r32, -r31)

        return theta4, theta6


