import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define system parameters
m1 = 1.0    # mass of the first link (kg)
m2 = 1.0    # mass of the second link (kg)
l1 = 1.0    # length of the first link (m)
l2 = 1.0    # length of the second link (m)

x = float(input('x = '))
y = float(input('y = '))

theta = np.arccos((x**2 + y**2 - l1**2 - l2**2)/2*l1*l2)
q1 = np.arctan2(y/x) - np.arctan2(l2*np.sin(theta)/(l1 + l2*np.cos(theta)))
q2 = q1 + theta
a = [0, l1*np.cos(q1), l1*np.cos(q1) + l2*np.cos(q2)]
b = [0, l1*np.sin(q1), l1*np.sin(q1) + l2*np.sin(q2)]
