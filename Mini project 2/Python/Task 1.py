from math import *
import numpy as np

# Lengths of links
l1 = 91.88     # length of link 1 (mm)
l2 = 104.54    # length of link 2 (mm)

# Let trajectory is a circle with equation (x-144.15)^2 + y^2 = 52.27^2
# So function becomes:
# x = 144.15 + 52.27*cos(t)
# y = 52.27*sin(t)
# where t is any value from 0 to 2*pi

t = np.linspace(0, 2*pi, 100)
x = 144.15 + 52.27 * np.cos(t)
y = 52.27 * np.sin(t)

# Function to calculate joint angles for given x, y positions.
def theta_desired(x_values, y_values):
    thetas = []
    for x, y in zip(x_values, y_values):
        if x == 0:
            x = 1e-10
        D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        theta2 = acos(D)
        theta1 = atan(y / x) - atan((l2 * sin(theta2)) / (l1 + (l2 * cos(theta2))))
        if abs(theta1) < 1e-5:
            theta1 = 0
        if abs(theta2) < 1e-5:
            theta2 = 0
        thetas.append((round(degrees(theta1),2), round(degrees(theta2),2)))
    return np.array(thetas)

# Calculate joint angles for the trajectory
theta_values = theta_desired(x, y)
print(theta_values)