import numpy as np
import matplotlib.pyplot as plt

timediv=200

#ellipse trajectory
a=2
b=2
elipsemega=1
smaple2=np.array(range(10,1,-1))
t=np.linspace(0,timediv,100)
y=a*np.cos(elipsemega*(np.pi/50)*t)
x=t-100
x=elipsemega*(np.pi/100)*x


l1=2
l2=2
m1=4
m2=2

trace = []
def inversekinematics(x,y):
    exp=(x*x+y*y)
    exp=exp-(l1*l1+l2*l2)
    exp=exp/(2*l1*l2)

    theta=np.arccos(exp)
    
    q1=np.arctan(y/x)-np.arctan((l2*np.sin(theta)/(l2*np.cos(theta)+l1)))+np.pi*(x<0)
    q2=q1+theta
    return q1,q2

fig,ax=plt.subplots()

ax.set_aspect('equal')

theta1,theta2=inversekinematics(x,y)
i=0
while True:
    i=i%np.size(theta1)

    plt.cla()

    end_effector_x=l1 * np.cos(theta1[i])+l2 * np.cos(theta2[i])
    end_effector_y=l1 * np.sin(theta1[i])+l2 * np.sin(theta2[i])

    trace.append((end_effector_x, end_effector_y))

    ax.plot([0, l1 * np.cos(theta1[i])], [0, l1 * np.sin(theta1[i])], 'b-')
    ax.plot([l1 * np.cos(theta1[i]), end_effector_x], [l1 * np.sin(theta1[i]), end_effector_y], 'b-')
    ax.plot(end_effector_x, end_effector_y, 'ro')
    ax.set_xlim(-l1-l2, l1+l2)
    ax.set_ylim(-l1-l2, l1+l2)
    ax.set_title("2-Link Manipulator with Elliptical Trajectory")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.plot(x,y)
    trace_x, trace_y = zip(*trace)
    ax.plot(trace_x, trace_y, 'k:', linewidth=0.5)
    plt.pause(0.01)
    i+=1

plt.show()
