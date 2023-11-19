import numpy as np

def inverse_kinematics_scara(x, y, z, l1, l2):
    # Compute q2 using the inverse trigonometric functions
    cos_q2 = (x**2 + y**2 + z**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sin_q2 = np.sqrt(1 - cos_q2**2)
    q2 = np.arctan2(sin_q2, cos_q2)

    # Compute q1 using the inverse trigonometric functions
    k1 = l1 + l2 * cos_q2
    k2 = l2 * sin_q2
    q1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return q1, q2

x = float(input("Enter x-coordinate of end-effector position: "))
y = float(input("Enter y-coordinate of end-effector position: "))
z = float(input("Enter z-coordinate of end-effector position: "))
l1 = float(input("Enter length of link 1 (l1): "))
l2 = float(input("Enter length of link 2 (l2): "))

q1, q2 = inverse_kinematics_scara(x, y, z, l1, l2)

print("\n" + "-" * 50 + "\n")
print("Inverse Position Kinematics:")

print("\tJoint Angle q1:", "{:.2f}".format(np.degrees(q1)), "degrees")
print("\tJoint Angle q2:", "{:.2f}".format(np.degrees(q2)), "degrees")
