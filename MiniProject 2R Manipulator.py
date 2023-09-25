#!/usr/bin/env python
# coding: utf-8

# Parth Deshpande 
# 21110151 

# TASK 1

# In[ ]:


import numpy as np
import matplotlib.pyplot as plt 
from scipy.integrate import odeint  #The scipy.integrate module provides functions for numerical integration, which is the process of approximating the definite integral of a function over a specified interval.
from matplotlib.animation import FuncAnimation #submodule of the Matplotlib library in Python that provides tools for creating animated visualizations.

# Defining the constats

m1 = 1.0
m2 = 1.0
l1 = 1.0
l2 = 1.0
t_max = 10.0   # total simulation time (s)
num_steps = 500

q1range = np.linspace(0, 2*(np.pi), num_steps)
q2range = np.linspace(0, 2*(np.pi), num_steps)


# let's Create animation

fig, ax = plt.subplots(figsize=(6, 6))  # Equal aspect ratio
ax.set_xlim(-0.75, 0.75)
ax.set_ylim(-0.75, 0.75)
line, = ax.plot([], [], 'o', markersize=10)

#defining the init fuction for the animation

def init(): #init() function is typically used to initialize the elements of the plot before the animation starts
    line.set_data([], []) # Initialize main line  
    return line

# defining the fuction for the animation

def animate(i):
    q1 = q1range[i]
    q2 = q2range[i]
    #using the equations derived in class 
    
    x = [0,l1*np.cos(q1) + l2*np.cos(q2),np.cos(q1)]
    y = [0,l1*np.sin(q1) + l2*np.sin(q2),np.sin(q1)]
    line.set_data(x,y)
    return line

ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True)

plt.xlabel('X') #label
plt.ylabel('Y')
plt.title('2R MANIPULATOR')

# Display the animation in the Jupyter Notebook
from IPython.display import HTML
HTML(ani.to_jshtml())


# TASK 3

# In[ ]:


import numpy as np 
import matplotlib.pyplot as plt

# Constants and parameters
l1 = 1.0  # Length of the first link
l2 = 1.0  # Length of the second link

# let's Create animation

fig, ax = plt.subplots(figsize=(6, 6))  # Equal aspect ratio
ax.set_xlim(-0.75, 0.75)
ax.set_ylim(-0.75, 0.75)
line, = ax.plot([], [], 'o', markersize=10)

#defining the init fuction for the animation

def init(): #init() function is typically used to initialize the elements of the plot before the animation starts
    line.set_data([], []) # Initialize main line  
    return line

# defining the fuction for the animation

def animate(i):
    q1 = q1range[i]
    q2 = q2range[i]
    #using the equations derived in class 
    
    x = [0,l1*np.cos(q1) + l2*np.cos(q2),np.cos(q1)]
    y = [0,l1*np.sin(q1) + l2*np.sin(q2),np.sin(q1)]
    line.set_data(x,y)
    return line


# TASK 4

# In[1]:


import numpy as np
import matplotlib.pyplot as plt

# Constants and parameters
l1 = 1.0  # Length of the first link
l2 = 1.0  # Length of the second link
q1_min, q1_max = np.degrees(35), np.degrees(145)  # Joint angle limits in degrees
q2_min, q2_max = np.degrees(35), np.degrees(145)  

# Generate joint angles within the specified limits
q1_values = np.linspace(q1_min, q1_max, 100)
q2_values = np.linspace(q2_min, q2_max, 100)

# Initialize arrays to store workspace coordinates
x_values = []
y_values = []

# Calculate workspace coordinates
for q1 in q1_values:
    for q2 in q2_values:
        x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
        y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
        x_values.append(x)
        y_values.append(y)

# Plot the workspace
plt.figure(figsize=(8, 6))
plt.scatter(x_values, y_values, s=5, c='b', label='Workspace')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.title('Workspace of the 2R Elbow Manipulator')
plt.xlim(-l1 - l2, l1 + l2)
plt.ylim(-l1 - l2, l1 + l2)
plt.axhline(0, color='k', linewidth=0.5)  # Add x-axis
plt.axvline(0, color='k', linewidth=0.5)  # Add y-axis
plt.legend()
plt.grid()
plt.show()


# In[ ]:




