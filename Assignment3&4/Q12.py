from math import degrees, acos, asin, sin

def inverse_kinematics(x, y, z, l1, l2, d):
    q2 = acos((z - l1) / d)
    q1 = asin(((-y * d * sin(q2)) - (x * l2)) / ((l2 ** 2) + ((d ** 2) * (sin(q2) ** 2))))

    q1 = round(degrees(q1), 2)
    q2 = round(degrees(q2), 2)

    return q1, q2

# Example usage:
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))
l1 = float(input("length of link 1 (l1) = "))
l2 = float(input("length of link 2 (l2) = "))
d = float(input("height of link 3 (d)  = "))

q1_result, q2_result = inverse_kinematics(x, y, z, l1, l2, d)

print("q1 = ", q1_result)
print("q2 = ", q2_result)

