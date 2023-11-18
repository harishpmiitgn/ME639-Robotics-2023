import numpy as np

def inverse_kinematics(x, y, z, l1, l2, d):
    # Compute joint angle q2 using the provided equation
    q2 = np.arccos((z - l1) / d)

    # Compute intermediate values for q1 calculation
    term1 = -y * d * np.sin(q2)
    term2 = x * l2

    # Compute joint angle q1 using the provided equation
    numerator = term1 - term2
    denominator = (l2**2) + ((d**2) * (np.sin(q2)**2))
    q1 = np.arcsin(numerator / denominator)

    return q1, q2

x = float(input("Enter x-coordinate of end-effector position: "))
y = float(input("Enter y-coordinate of end-effector position: "))
z = float(input("Enter z-coordinate of end-effector position: "))
l1 = float(input("Enter length of link 1 (l1): "))
l2 = float(input("Enter length of link 2 (l2): "))
d = float(input("Enter distance between links 2 and 3 (d): "))

q1, q2 = inverse_kinematics(x, y, z, l1, l2, d)


print("\n" + "-" * 50 + "\n")
print("Inverse Position Kinematics:")

print("\tJoint Angle q1:", "{:.2f}".format(np.degrees(q1)), "degrees")
print("\tJoint Angle q2:", "{:.2f}".format(np.degrees(q2)), "degrees")
