import numpy as np
import matplotlib.pyplot as plt
l1=1
l2=1
l3=1
phi=np.deg2rad(90)

def fwd_k(theta):
    theta1,theta2,theta3=theta
    x=l1*np.cos(theta1)+l2*np.cos(theta1+theta2)+l3*np.cos(theta1+theta2+theta3)
    y=l1*np.sin(theta1)+l2*np.sin(theta1+theta2)+l3*np.sin(theta1+theta2+theta3)
    return x,y

def jacobian(theta):
    theta1,theta2,theta3=theta
    J=np.zeros((2,3))
    J[0][0]=-l1*np.sin(theta1)-l2*np.sin(theta1+theta2)-l3*np.sin(theta1+theta2+theta3)
    J[1][0]=l1*np.cos(theta1)+l2*np.cos(theta1+theta2)+l3*np.cos(theta1+theta2+theta3)
    J[0][1]=-l2*np.sin(theta1+theta2)-l3*np.sin(theta1+theta2+theta3)
    J[1][1]=l2*np.cos(theta1+theta2)+l3*np.cos(theta1+theta2+theta3)
    J[0][2]=-l3*np.sin(theta1+theta2+theta3)
    J[1][2]=l3*np.cos(theta1+theta2+theta3)
    return J
#trajectory
omega=1
r=1.5
points=200
timestep=0.1
def trajectory(t):
    x=r*np.cos(omega*t)
    y=r*np.sin(omega*t)
    return x,y

def velocity(t):
    Vx=-omega*r*np.sin(omega*t)
    Vy=omega*r*np.cos(omega*t)
    return np.array([Vx,Vy])

def ik(x,y,phi):
    x2=x-l3*np.cos(phi)
    y2=y-l3*np.sin(phi)
    theta2=(x2*x2+y2*y2-l1*l1-l2*l2)
    theta2=theta2/(2*l1*l2)
    theta2=np.arccos(theta2)
    theta1=np.arctan2(y2,x2)-np. arctan( (l2*np. sin (theta2) / ( l2*np. cos (theta2)+l1) ))
    theta3=phi-theta1-theta2
    return theta1,theta2,theta3

fig,ax=plt.subplots()
trace=[]
circle=[]

def display(theta):
    plt.cla()
    theta1,theta2,theta3=theta
    end_effector_x,end_effector_y=fwd_k(theta)

    trace.append((end_effector_x, end_effector_y))

    ax.plot([0, l1 * np.cos(theta1)], [0, l1 * np.sin(theta1)], 'b-')
    ax.plot([l1 * np.cos(theta1),l1*np.cos(theta1)+l2*np.cos(theta1+theta2)], [l1 * np.sin(theta1), l1*np.sin(theta1)+l2*np.sin(theta1+theta2)], 'b-')
    ax.plot([l1*np.cos(theta1)+l2*np.cos(theta1+theta2),end_effector_x], [l1*np.sin(theta1)+l2*np.sin(theta1+theta2), end_effector_y], 'b-')

    ax.plot(l1 * np.cos(theta1), l1 * np.sin(theta1), 'ro')
    ax.plot(l1*np.cos(theta1)+l2*np.cos(theta1+theta2), l1*np.sin(theta1)+l2*np.sin(theta1+theta2), 'ro')
    
    
    ax.plot(end_effector_x, end_effector_y, 'ro')

    ax.set_xlim(-l1-l2-l3, l1+l2+l3)
    ax.set_ylim(-l1-l2-l3, l1+l2+l3)
    ax.set_title("3-Link Manipulator with Circular Trajectory")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    xd,yd=trajectory(t)
    ax.plot(xd,yd)
    circle.append((xd,yd))
    
    circlex,circley=zip(*circle)
    trace_x, trace_y = zip(*trace)
    ax.plot(circlex,circley)
    ax.plot(trace_x, trace_y, 'k:', linewidth=0.5)
    plt.pause(0.1)
    
i=0

#t=np.linspace(0,timeend*1000,1000)
# theta=ik(1.5,0,np.pi/2)
# print(fwd_k(theta))
theta=[np.pi/2,0,0]
theta=np.array(theta)
i=0
t=0
kp=1
while i<100000000:
    print(i)
    J=jacobian(theta)
    display(theta)

    x,y=fwd_k(theta)
    xd,yd=trajectory(t)
    error=np.array([x-xd,y-yd])
    theta=theta+timestep*np.matmul(np.linalg.pinv(J),(velocity(t)-kp*error))
    print(theta)
    i+=1
    t+=timestep
