import numpy as np
import matplotlib.pyplot as plt
from math import tan

# Define link lengths (in arbitrary units)
l1 = 2
l2 = 1

# Define joint angle limits in radians
theta1_min = np.deg2rad(35)
theta1_max = np.deg2rad(145)
theta2_min = np.deg2rad(35)
theta2_max = np.deg2rad(145)

# Create a grid of joint angles
theta1_vals = np.linspace(theta1_min, theta1_max, 50)
theta2_vals = np.linspace(theta2_min, theta2_max, 50)
theta2_grid, theta1_grid = np.meshgrid(theta2_vals, theta1_vals)

# Calculate end effector positions
x = l1 * np.cos(theta1_grid) + l2 * np.cos(theta2_grid)
y = l1 * np.sin(theta1_grid) + l2 * np.sin(theta2_grid)


# Plot the workspace
plt.figure(figsize=(8, 6))
plt.scatter(x, y, c='black', marker='.',s=20)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Workspace of 2R Manipulator Robot')
plt.xlim(-3,3)
plt.ylim(0, 3.2)
plt.grid(True)

# Define the coordinates of the endpoints of the line 35 degree
line1_x = [0, 3]
line1_y = [0, 3*tan(0.610865238)]

# Define the coordinates of the endpoints of the line 145 degree
line2_x = [0, -3]
line2_y = [0, -3*tan(2.53072742)]

# Plot the straight line
plt.plot(line1_x, line1_y, color='blue', linestyle='--', linewidth=1)
plt.plot(line2_x, line2_y, color='blue', linestyle='--', linewidth=1)

plt.show()
