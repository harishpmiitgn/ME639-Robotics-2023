import numpy as np

def calc_j(theta1,theta2,a1,a2):

    q1=np.radians(theta1)
    q2=np.radians(theta2)
    j=np.zeros(6,3)
    j[5][0]=j[5][1]=1
    j[2][2]=-1
    j[0][0]=-(a2*np.sin(q1+q2)+a1*np.sin(q1))
    j[1][0]=a2*np.cos(q1+q2)+a1*np.cos(q1)

    j[0][1]=-(a2*np.sin(q1+q2))
    j[1][1]=a2*np.cos(q1+q2)
    return j

J=calc_j(20,50,3,5)

print(J)