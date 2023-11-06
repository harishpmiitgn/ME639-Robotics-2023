import math
import numpy as np

def stanford_manipulator_inverse_kinematics(px, py, pz, l1, l2, l3):
    # Calculating theta1
    if px == 0 and py == 0:
        theta1 = 0  # or any other value, since it's arbitrary
    else:
        theta1 = math.atan2(py, px)

    # Calculating r and s
    r = (px ** 2 + py ** 2) ** 0.5
    s = pz - l1

    # Calculating theta2
    theta2 = math.atan2(r, s)

    # Calculating d2
    d2 = (r ** 2 + s ** 2) ** 0.5 - l2 - l3

    return theta1, theta2, d2

# can be verified using the forward kinematics derived in Q4