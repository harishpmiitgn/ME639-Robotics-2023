import numpy as np
import task3 as tk
def jacobian(theta,H,d5):
    theta1,theta2,theta3,theta4,theta5=theta
    J=np.zeros((6,5))
    endeff=tk.endeff(H,d5)[:3]
    O=np.eye(4)
    for i in range(5):
        O=np.matmul(O,H[i])
        zaxis=O[2,:3]
        J[3:,i]=zaxis
        vel=endeff-O[:3,3]
        vel=np.cross(zaxis,vel)
        J[:3,i]=vel
    return J

l=np.ones(5)
off=np.ones(4)
theta =[30,20,50,52,45]
print(jacobian(theta,tk.Hcalc(theta,l,off),[0,0,5,1]))
        