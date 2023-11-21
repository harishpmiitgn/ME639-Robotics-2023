from math import *

def inversekinematics(x, y, z, roll, pitch, yaw):
    # for 6-DOF robotic arm with a spherical wrist

    
    d1 = 2
    l2 = 3
    l3 = 4
    d6 = 5

    # Wrist position
    Wrist_x = x - d6 * cos(yaw) * cos(pitch)
    Wrist_y = y - d6 * sin(yaw) * cos(pitch)
    Wrist_z = z + d6 * sin(pitch)

    # Inverse kinematics for the first three joints
    angle1 = atan2(Wrist_y, Wrist_x)

    d = sqrt(Wrist_x**2 + Wrist_y**2 + (Wrist_z - d1)**2)
    a = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)
    beta = atan2(sqrt(1 - a**2), a)

    angle3 = pi - beta - atan2(l3 * sin(beta), l2 + l3 * cos(beta))

    k1 = l2 + l3 * cos(beta)
    k2 = l3 * sin(beta)
    thetl2 = atan2(Wrist_z - d1, sqrt(Wrist_x**2 + Wrist_y**2)) - atan2(k2, k1)

    # Inverse kinematics for the last three joints of wrist
    angle4 = roll - thetl2 - angle3

    # Ensure angle4 is within -pi to pi range
    angle4 = (angle4 + pi) % (2 * pi) - pi

    angle5 = pitch - thetl2 - angle3

    angle6 = yaw - angle1

    return angle4, angle5, angle6

# Example usage

print("co-ordinates of end effector ")
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))

roll = radians(30)
pitch = radians(45)
yaw = radians(60)

angle4, angle5, angle6 = inversekinematics(x, y, z, roll, pitch, yaw)
print("θ4:", degrees(angle4))
print("θ5:", degrees(angle5))
print("θ6:", degrees(angle6))
