import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

L1 =int(input("enter length of link 1= "))
L2=int(input("enter length of link 2= "))

a =float(input("enter x intercept of wall= "))
b=float(input("enter y intercept of wall= "))
z=float((a*b)/(a+b))
Nx=10*np.cos(np.arctan(abs(b/a)))
Ny=10*np.sin(np.arctan(abs(b/a)))

start_point = np.array([L1-L2, 0])  # Starting position
end_point = np.array([z,z])  # Ending position
trajectory = np.linspace(start_point, end_point, 25)
p="point of contact : "+str(z)+","+str(z)

# Initialize plot
fig, ax = plt.subplots()
ax.set_xlim(-a*5, a*5)
plt.grid()
ax.set_ylim(-b*5, b*5)
line, = ax.plot([], [], 'o-', lw=2)
wall=plt.plot([0,b],[a,0],'g-',linewidth=3)


def init():
    line.set_data([], [])
    return line,

def animate(i):
    target_point = trajectory[i]
    target_x, target_y = target_point

    theta1, theta2 = inverse_kinematics(target_x, target_y)
    manipulator_x = [0, L1 * np.cos(theta1), L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)]
    manipulator_y = [0, L1 * np.sin(theta1), L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)]

    line.set_data(manipulator_x, manipulator_y)
    return line,


def inverse_kinematics(x, y):
    theta2 = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    theta1 = np.arctan2(y,x) - np.arctan2(L2 * np.sin(theta2),( L1 + L2 * np.cos(theta2)))
    return theta1, theta2

if (z>(L1+L2)):
    print ("wall is beyond reach of manipulator") #rough estimate
else :
    print ("cordinates of point chosen to apply normal force of 10 N on wall is \n(",z,z,")")
    theta1, theta2 = inverse_kinematics(z,z)
    torque1=Ny*L1*np.cos(theta1)-Nx*L1*np.sin(theta1)
    torque2=Ny*L2*np.cos(theta1)-Nx*L2*np.sin(theta1)
    print("The required torque of motor 1 and motor 2 are : \n ",torque1,torque2)
    # Animate the manipulator following the trajectory
    ani = FuncAnimation(fig, animate, frames=len(trajectory), init_func=init, blit=True, repeat=False)
    plt.text(-2*a,2*b,"normal force is 10 N")
    plt.text(-2*a,2.5*b,p)
    plt.show()
    
