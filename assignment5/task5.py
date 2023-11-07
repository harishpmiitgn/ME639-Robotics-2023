import numpy as np
#for 6dof manipulator
def spherical(r36):
    psi=0
    if r36[0, 2] != 0 or r36[1, 2] !=0 :
        theta = np.arctan2(-np.sqrt(1 - r36[2, 2]**2), r36[2, 2])
        phi = np.arctan2(-r36[1, 2],-r36[0, 2])
        si = np.arctan2(-r36[2, 1], r36[2, 0])
    else:
        phi = 0.
        if r36[2, 2] > 0 :
            theta = 0
            phi = np.arctan2(r36[1, 0], r36[0, 0])
        else :
            theta = np.pi
            phi = - np.arctan2(-r36[0, 1],-r36[0, 0])
    return np.array([phi, theta, psi])
