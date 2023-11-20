import numpy as np

def calc_r(theta,axi):
    t=np.radians(theta)
    if axi=='z':        
        h=np.zeros((3,3))
        h[0][0]=np.cos(t)
        h[0][1]=-np.sin(t)
        h[1][0]=np.sin(t)
        h[1][1]=np.cos(t)
        h[2][2]=1
    elif axi=='y':
        h=np.zeros((3,3))
        h[0][0]=np.cos(t)
        h[0][2]=np.sin(t)
        h[2][0]=-np.sin(t)
        h[2][2]=np.cos(t)
        h[1][1]=1
    elif axi=='x':
        h=np.zeros((3,3))
        h[1][1]=np.cos(t)
        h[1][2]=-np.sin(t)
        h[2][1]=np.sin(t)
        h[2][2]=np.cos(t)
        h[0][0]=1
    return h
def calc_h(r,d):
    h=np.zeros((4,4))
    for i in range(3):
        for j in range(3):
            h[i][j]=r[i][j]
    h[3][3]=1
    for i in range(3):
        h[i][3]=d[i]
    
    return h

def end_eff(theta,L,off):
    theta1,theta2,theta3,theta4,theta5=theta
    l1,l2,l3,l4,l5=L
    off2,off3,off4,off5=off

    r01=calc_r(theta1,'z')
    r12=np.matmul(calc_r(np.pi/2,'x'),calc_r(theta2,'z'))
    r23=calc_r(theta3,'z')
    r34=calc_r(theta4,'z')
    r45=np.matmul(calc_r(np.pi/2,'y'),calc_r(theta5,'z'))
    H01=calc_h(r01,[0,0,l1])
    H12=calc_h(r12,[l2,0,off2])
    H23=calc_h(r23,[l3,0,off3])
    H34=calc_h(r34,[l4,0,off4])
    H45=calc_h(r45,[off5,0,l5])

    return H01,H12,H23,H34,H45

l=np.ones(5)
off=np.ones(4)
theta =[30,20,50,52,45]
theta=np.deg2rad(theta)

h=end_eff(theta,l,off)
print(h)
