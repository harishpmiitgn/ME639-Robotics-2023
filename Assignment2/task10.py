import numpy as np

def calc_j(theta1,theta2,theta3,a1,a2,a3):

    q1=np.radians(theta1)
    q2=np.radians(theta2)
    q3=np.radians(theta3)
    j=np.zeros(6,3)
    j[5][0]=j[5][1]=j[5][2]=1
    
    j[0][0]=-(a3*np.sin(q1+q2+q3)+a2*np.sin(q1+q2)+a1*np.sin(q1))
    j[1][0]=a3*np.cos(q1+q2+q3)+a2*np.cos(q1+q2)+a1*np.cos(q1)
    j[0][1]=-a3*np.sin(q1+q2+q3)+a2*np.sin(q1+q2)
    j[1][1]=a3*np.cos(q1+q2+q3)+a2*np.cos(q1+q2)
    j[0][2]=-a3*np.sin(q1+q2+q3)
    j[1][2]=a3*np.cos(q1+q2+q3)

    return j

J=calc_j(20,50,25,3,5,8)

print(J)
