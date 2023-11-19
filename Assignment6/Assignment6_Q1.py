import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics(theta):
    theta1,theta2,theta3=theta
    x = l1*np.cos(theta1)+l2*np.cos(theta1+theta2)+l3*np.cos(theta1+theta2+theta3)
    y = l1*np.sin(theta1)+l2*np.sin(theta1+theta2)+l3*np.sin(theta1+theta2+theta3)
    return x,y

def jacobian(theta):
    theta1,theta2,theta3=theta
    J=np.zeros((2,3))
    J[0][0] = -l1*np.sin(theta1)-l2*np.sin(theta1+theta2)-l3*np.sin(theta1+theta2+theta3)
    J[0][1] = -l2*np.sin(theta1+theta2)-l3*np.sin(theta1+theta2+theta3)
    J[0][2] = -l3*np.sin(theta1+theta2+theta3)
    J[1][0] = l1*np.cos(theta1)+l2*np.cos(theta1+theta2)+l3*np.cos(theta1+theta2+theta3)
    J[1][1] = l2*np.cos(theta1+theta2)+l3*np.cos(theta1+theta2+theta3)
    J[1][2] = l3*np.cos(theta1+theta2+theta3)
    return J

def trajectory(t):
    x=r*np.cos(omega*t)
    y=r*np.sin(omega*t)
    return x,y

def velocity(t):
    Vx=-omega*r*np.sin(omega*t)
    Vy=omega*r*np.cos(omega*t)
    return np.array([Vx,Vy])

def plot(theta):
    plt.cla()
    theta1,theta2,theta3=theta
    end_effector_x,end_effector_y=forward_kinematics(theta)

    trace.append((end_effector_x, end_effector_y))

    ax.plot([0, l1 * np.cos(theta1)], [0, l1 * np.sin(theta1)], 'b-')
    ax.plot([l1 * np.cos(theta1),l1*np.cos(theta1)+l2*np.cos(theta1+theta2)], [l1 * np.sin(theta1), l1*np.sin(theta1)+l2*np.sin(theta1+theta2)], 'b-')
    ax.plot([l1*np.cos(theta1)+l2*np.cos(theta1+theta2),end_effector_x], [l1*np.sin(theta1)+l2*np.sin(theta1+theta2), end_effector_y], 'b-')

    ax.plot(l1 * np.cos(theta1), l1 * np.sin(theta1), 'ro')
    ax.plot(l1*np.cos(theta1)+l2*np.cos(theta1+theta2), l1*np.sin(theta1)+l2*np.sin(theta1+theta2), 'ro')
    
    
    ax.plot(end_effector_x, end_effector_y, 'ro')

    ax.set_xlim(-l1-l2-l3, l1+l2+l3)
    ax.set_ylim(-l1-l2-l3, l1+l2+l3)
    ax.set_title("Circular Trajectory Using a 3-Link Manipulator")
    xd,yd=trajectory(t)
    ax.plot(xd,yd)
    circle.append((xd,yd))
    
    circlex,circley=zip(*circle)
    trace_x, trace_y = zip(*trace)
    ax.plot(circlex,circley)
    ax.plot(trace_x, trace_y, 'k:', linewidth=0.5)
    plt.pause(0.1)

#manipulator
l1 = 1 # length in m
l2 = 1 # length in m
l3 = 1 # length in m
phi=np.deg2rad(90)

#trajectory
omega=1
r=1.5 # radius in meters
points=500
timestep=0.1

theta=[np.pi/2,0,0]
theta=np.array(theta)
i=0
t=0
kp=1


fig,ax=plt.subplots()
trace=[]
circle=[]

while i<150:
    print(i, end=":")
    J=jacobian(theta)
    plot(theta)

    x,y=forward_kinematics(theta)
    xd,yd=trajectory(t)
    error=np.array([x-xd,y-yd])
    theta=theta+timestep*np.matmul(np.linalg.pinv(J),(velocity(t)-kp*error))
    print(theta)
    i+=1
    t+=timestep