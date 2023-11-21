from math import 
print("co-ordinates of end effector with base frame")
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))

D1 = float(input(" Hight link 1 = "))
L1 = float(input("length link 1  = "))
L2 = float(input("length link 2  = "))
d = D1 - z

q2 = acos ((x**2+y**2-L1**2-L2**2)/(2*L1*L2))
q1 = atan(y/x) - atan((L2*sin(q2))/(L1+L2*cos(q2)))

q2 = round(degrees(q2),2)
q1 = round(degrees(q1),2)


print("q1 = ",q1)
print("q2 = ",q2)


