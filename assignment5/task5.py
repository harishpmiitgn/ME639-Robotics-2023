import numpy as np
#for 6dof manipulator
def spherical(r36):
    theta5=np.arccos(r36[2,2])
    if abs(r36[2,2])!=1 :
        theta4 = np.arctan2(-r36[1, 2],-r36[0, 2])
        theta6 = np.arctan2(-r36[2, 1], r36[2, 0])
    else:
        theta6 = 0
        if r36[2, 2] > 0 :
            theta4 = np.arctan2(r36[1, 0], r36[0, 0])
        else :
            theta4 = - np.arctan2(-r36[0, 1],-r36[0, 0])
    return np.array([theta4, theta5, theta6])
