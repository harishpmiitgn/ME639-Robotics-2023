from math import *

px = float(input("Px = "))
py = float(input("Py = "))
pz = float(input("Pz = "))

# Link lengths
l1 = 0.25
l2 = 0.25
l3 = 0.25

# Check for division by zero
if px != 0:
    q1 = atan(py / px)
else:
    # Handle the case where px is zero to avoid division by zero
    if py>=0:
        q1 = pi / 2  
    else: 
        q1 = -pi / 2
    
q3 = acos((px**2+py**2+(pz-l1)**2-l2**2-l3**2)/(2*l2*l3))

if px==0 and py==0:
    q2 = pi/2
else:
    q2 = atan((pz-l1)/(px**2+py**2)**0.5)-atan((l3*sin(q3))/(l2+l3*cos(q3)))


q1 = degrees(q1)
q2 = degrees(q2)
q3 = degrees(q3)

print("\nInverse kinematics:")
print("The angles (degrees) are:")
print("q1 = ",q1)
print("q2 = ",q2)
print("q3 = ",q3)

print("\nUsing same angle values in forward kinematics")
print("Position of end effector:")

q1 = radians(q1)
q2 = radians(q2)
q3 = radians(q3)

px = round((l3*cos(q1)*cos(q2+q3))+(l2*cos(q1)*cos(q2)),2)
py = round((l3*sin(q1)*cos(q2+q3))+(l2*sin(q1)*cos(q2)),2)
pz = round((l3*sin(q2+q3))+(l2*sin(q2))+l1,2)

print("Px =", px)
print("Py =", py)
print("Pz =", pz)
