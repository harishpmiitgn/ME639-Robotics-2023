import numpy as np

l1=0.25
l2=0.25
l3=0.25

m1=0.8
m2=0.8
m3=0.8

I1=0.005
I2=0.005
I3=0.005

def Dh(d,theta,alfa,a):
    dh=np.zeros((4,4))
    dh[3][3]=1
    dh[0][0]=np.cos(theta)
    dh[1][0]=np.sin((theta))
    dh[0][1]=-np.sin(theta)*np.cos(alfa)
    dh[1][1]=np.cos(theta)*np.cos(alfa)
    dh[2][1]=np.sin(alfa)
    dh[0][2]=np.sin(theta)*np.sin(alfa)
    dh[1][2]=-np.cos(theta)*np.sin(alfa)
    dh[2][2]=np.cos(alfa)
    dh[0][3]=a*np.cos(theta)
    dh[1][3]=a*np.sin(theta)
    dh[2][3]=d
    return dh
#SCARA
def ik(x,y,z):
    theta2=x**2+y**2-l1**2-l2**2
    theta2=theta2/(2*l1*l2)
    theta2=np.arccos(theta2)
    theta1=np.arctan2(y,x)-np.arctan2(l2*np.sin(theta2),(l1+l2*np.cos(theta2)))

    d3=-z

    return theta1,theta2,d3

def fk(q):
    theta1,theta2,d3=q
    T01=Dh(0,theta1,0,l1)
    T12=Dh(0,theta2,np.pi,l2)
    T23=Dh(d3,0,0,0)

    T02=np.matmul(T01,T12)
    T03=np.matmul(T02,T23)
    x,y,z=T03[0:3,3]
    return x,y,z
A=[.4  ,0.06,.1]
B=[.4,.01,.1]
C=[.35,.01,.1]
D=[.35,.06,.1]

print("For A")
q=ik(A[0],A[1],A[2])
print(fk(q))

print("For B")
q=ik(B[0],B[1],B[2])
print(fk(q))

print("For C")
q=ik(C[0],C[1],C[2])
print(fk(q))

print("For D")
q=ik(D[0],D[1],D[2])
print(fk(q))
