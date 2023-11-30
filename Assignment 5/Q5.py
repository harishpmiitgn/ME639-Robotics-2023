import numpy as np
from math import atan2, sqrt

def inverse_spherical_wrist(U):

    u12, u13, u22, u23, u31, u32, u33, u11, u21 = U[0,1], U[0, 2], U[1, 1], U[1, 2], U[2, 0], U[2, 1], U[2, 2], U[0, 0], U[1, 0]

    c1 = np.arctan2(u23, u33)
    s1 = np.arctan2(-u13, np.sqrt(u23 ** 2 + u33 ** 2))

    c5 = np.cos(s1)
    if abs(c5) > 1e-6:
        s5 = u33 / c5
        theta5 = np.arctan2(s1 * u13 - c1 * u23, c1 * u13 + s1 * u23 - c5)

        theta4 = np.arctan2(c1 * u23 - s1 * u13, -c1 * u33 + s1 * u23)

        theta6 = np.arctan2(-s1 * u12 + c1 * u22, s1 * u11 - c1 * u21)

        return theta4, theta5, theta6
    else:

        theta4 = np.arctan2(u21, u11)
        theta6 = np.arctan2(u32, -u31)

        return theta4, theta6


