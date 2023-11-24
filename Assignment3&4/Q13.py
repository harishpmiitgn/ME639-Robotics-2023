from math import degrees, acos, atan, sin,cos

def inverse_kinematics_scara(x, y, z, d1, l1, l2):
    d = d1 - z
    q2 = acos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
    q1 = atan(y / x) - atan((l2 * sin(q2)) / (l1 + l2 * cos(q2)))

    q1 = round(degrees(q1), 2)
    q2 = round(degrees(q2), 2)

    return q1, q2

# Example usage:
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))
d1 = float(input("height of link 1 (d1)  = "))
l1 = float(input("length of link 1 (l1) = "))
l2 = float(input("length of link 2 (l2) = "))

q1_result, q2_result = inverse_kinematics_scara(x, y, z, d1, l1, l2)

print("q1 = ", q1_result)
print("q2 = ", q2_result)


