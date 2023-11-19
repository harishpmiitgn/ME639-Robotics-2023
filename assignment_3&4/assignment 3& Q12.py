import numpy as np

def inverse_kinematics_RRP(link1, link2, link3, x, y, z):
    # Calculate the first joint angle (theta1)
    theta1 = np.arctan2(y, x)

    # Calculate the distance from the base to the end-effector projection in the xz plane
    r = np.sqrt(x**2 + z**2)

    # Calculate the angle between link2 and the projection of the end-effector in the xz plane
    alpha = np.arctan2(z, x)

    # Using the Law of Cosines to calculate the angle between link2 and link3 (theta2)
    cos_theta2 = (link2**2 + r**2 - link3**2) / (2 * link2 * r)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = alpha - np.arctan2(sin_theta2, cos_theta2)

    # Calculate the prismatic displacement (d3) along the common axis of link2 and link3
    d3 = z - link2 * np.sin(theta2)

    return theta1, theta2, d3

def forward_kinematics_RRP(link1, link2, link3, theta1, theta2, d3):
    # Calculate the position of the end of link2 (W)
    Wx = link2 * np.cos(theta1) * np.cos(theta2)
    Wy = link2 * np.sin(theta1) * np.cos(theta2)
    Wz = link2 * np.sin(theta2)

    # Calculate the position of the end-effector (E)
    Ex = Wx
    Ey = Wy
    Ez = Wz + d3

    return Ex, Ey, Ez

# Example usage:
link1 = 2.0  # Length of the first link
link2 = 2.0  # Length of the second link
link3 = 2.0  # Length of the third link
x = 1.5     # Desired end-effector x-coordinate
y = 0.5     # Desired end-effector y-coordinate
z = 1.0     # Desired end-effector z-coordinate
print("sample values : \nlink lengths: 2.0,2.0,2.0 \ndesired coordinates: 1.5,0.5,1.0")
theta1, theta2, d3 = inverse_kinematics_RRP(link1, link2, link3, x, y, z)
print(f"Theta1 (in radians): {theta1}")
print(f"Theta2 (in radians): {theta2}")
print(f"Prismatic Displacement (d3): {d3}")


x, y, z = forward_kinematics_RRP(link1, link2, link3, theta1, theta2, d3)
print(f"End-effector x-coordinate: {x}")
print(f"End-effector y-coordinate: {y}")
print(f"End-effector z-coordinate: {z}")
