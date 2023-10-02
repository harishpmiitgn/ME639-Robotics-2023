import numpy as np
#jacobian of 2R manipulator 

def jacobian(theta1,theta2,l1,l2):

    J = np.array([[-l1*np.sin(theta1)-l2*np.sin(theta1+theta2),-l2*np.sin(theta1+theta2)],
                  [l1*np.cos(theta1)+l2*np.cos(theta1+theta2),l2*np.cos(theta1+theta2)]])
    
    return J