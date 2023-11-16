import numpy as np
import matplotlib.pyplot as plt

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

    return np.array([theta1,theta2,d3])

def fk(q):
    theta1,theta2,d3=q
    T01=Dh(0,theta1,0,l1)
    T12=Dh(0,theta2,np.pi,l2)
    T23=Dh(d3,0,0,0)

    T02=np.matmul(T01,T12)
    T03=np.matmul(T02,T23)
    x,y,z=T03[0:3,3]
    return np.array(x,y,z)
A=np.array([.4  ,0.06,.1])
B=np.array([.4,.01,.1])
C=np.array([.35,.01,.1])
D=np.array([.35,.06,.1])
points=np.array([A,B,C,D])
timetaken=100
t_smol=np.linspace(0,timetaken,timetaken)

def genlinear_trajectory(initial,final,t):
    traj=initial+(final-initial)*t/timetaken
    return traj
theta1=[]
theta2=[]
d3=[]
xe=[]
ye=[]
ze=[]
for i in range(4):
    for t in t_smol:
        x,y,z=genlinear_trajectory(points[i,:],points[(i+1)%4,:],t)
        xe.append(x)
        ye.append(y)
        ze.append(z)
        q=ik(x,y,z)
        theta1.append(q[0])
        theta2.append(q[1])
        d3.append(q[2])

tnet=np.linspace(0,4*timetaken,4*timetaken)
##################################################################
##################################################################
fig, axs = plt.subplots(2, 3, figsize=(15, 10), sharex=True)

# Plot xe, ye, ze, theta1, theta2, d3 against tnet
axs[0, 0].plot(tnet, xe, label='xe')
axs[0, 1].plot(tnet, ye, label='ye')
axs[0, 2].plot(tnet, ze, label='ze')
axs[1, 0].plot(tnet, np.rad2deg(theta1), label='theta1')
axs[1, 1].plot(tnet, np.rad2deg(theta2), label='theta2')
axs[1, 2].plot(tnet, d3, label='d3')

# Set labels and title
axs[1, 0].set_xlabel('Time')
axs[1, 1].set_xlabel('Time')
axs[1, 2].set_xlabel('Time')
fig.suptitle('SCARA Robot Trajectory')

# Show legend
for ax_row in axs:
    for ax in ax_row:
        ax.legend()

# Display the plots
plt.show()