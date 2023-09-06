#Arya|20110025
import numpy as np
from position_UR5 import position_UR5
#jacobian for UR5 

def jacobian (theta1,theta2,theta3,theta4,theta5,theta6,l1,l2,l3,l4,l5,l6,l7,l8):
    [x1,x2,x3]=position_UR5(theta1,theta2,theta3,theta4,theta5,theta6,l1,l2,l3,l4,l5,l6,l7,l8)
    x1_dot = np.diff(x1)
    x2_dot = np.diff(x2)
    x3_dot = np.diff(x3)
    theta_dot = np.array([np.diff(theta1),np.diff(theta2),np.diff(theta3),np.diff(theta4),np.diff(theta5),np.diff(theta6)]).T
    
