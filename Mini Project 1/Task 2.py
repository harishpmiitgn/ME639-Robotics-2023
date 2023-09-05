import numpy as np
from math import atan,acos
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#In this case we consider a wall and plan the trajectory of the end effector to be normal to the wall
#Lets consider the line y = -0.75 + 20

x_w = np.linspace(0, 20, 100)
y_w = -0.75*x_w + 20

#then the trajectory will be parallel to y = (4/3)x, here we consider y = (4/3)x + 10/3 for better graph.
x_t = np.linspace(5, 8, 100)
y_t = (4/3)*x_t + 10/3

#Creating length of arms
l1 = 10
l2 = 10

#Creating coordinates of base joint(joint 1) and joint 2 and end eff
x0 = [0]*100
y0 = [0]*100

x1 = [0]*100
y1 = [0]*100

x_end = [0]*100
y_end = [0]*100

for i in range(100):
  theta = acos((x_t[i]**2 + y_t[i]**2 - l1**2 - l2**2)/(2*l1*l2))
  q1 = atan(y_t[i]/x_t[i]) - atan((l2*np.sin(theta))/(l1 + l2*np.cos(theta)))
  # q1 is the angle at which link 1 is inclined
  x1[i] = l1*np.cos(q1)
  y1[i] = l1*np.sin(q1)

  q2 = q1 + theta
  x_end[i] = x1[i] + l2*np.cos(q2)
  y_end[i] = y1[i] + l2*np.sin(q2)

#Creating the plots
fig, graph = plt.subplots()
graph.set_xlim(-5, 25)
graph.set_ylim(-5, 25)
graph.grid(True)
graph.set_aspect('equal', adjustable='box')

link_1, = graph.plot([],[], 'b-', linewidth=3)
link_2, = graph.plot([],[], 'b-', linewidth=2)
joint_1, = graph.plot([],[], 'go', markersize=10)
joint_2, = graph.plot([],[], 'go', markersize=10)
end_eff, = graph.plot([],[], 'ro', markersize=10)
trajectory, = graph.plot([],[], 'k--', linewidth=1)

traced_x=[]
traced_y=[]

def init():
  link_1.set_data([], [])
  link_2.set_data([], [])
  joint_1.set_data([], [])
  joint_2.set_data([], [])
  end_eff.set_data([], [])
  trajectory.set_data([], [])
  return link_1, link_2, joint_1, joint_2, end_eff, trajectory

def animate(i):
  link_1.set_data([x0[i], x1[i]], [y0[i], y1[i]])
  link_2.set_data([x1[i], x_end[i]], [y1[i], y_end[i]])
  joint_1.set_data(x0[i], y0[i])
  joint_2.set_data(x1[i], y1[i])
  end_eff.set_data(x_t[i], y_t[i])

  traced_x.append(x_t[i])
  traced_y.append(y_t[i])
  trajectory.set_data(traced_x, traced_y)

  return link_1, link_2, joint_1, joint_2, end_eff, trajectory

graph.plot(x_w, y_w, 'c-', linewidth = 5)
ani = FuncAnimation(fig, animate, frames=100, init_func=init, blit=True)
plt.show()