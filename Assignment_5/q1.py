import numpy as np

print("please enter end effector coordinates ")
x=float(input())
y=float(input())
z=float(input())

print("please enter the link lengths")
l1=float(input())
l2=float(input())
l3=float(input())

q1 = np.arctan2(x,y)

d = np.sqrt(x**2+y**2+(z-l1)**2)-(l2+l3)

q2 = np.arctan2(z-l1, np.sqrt(x**2+y**2))

print(f" inverse solutions are q1 = {np.degrees(q1)}, q2 = {np.degrees(q2)}, d = {d}")

# coordinate verification

def getH(R, d):
    H=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H

def getR(q):
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

H01=getH(getR(q1), [[0],[0],[0]])
H12=getH(np.matmul([[1, 0, 0],[0, 0, -1],[0, 1, 0]], getR(q2)), [[0],[0],[l1]])
H23=getH(getR(0), [[l2+l3],[0],[0]])
P=np.linalg.multi_dot([H01,H12,H23,[[d],[0],[0],[1]]])

print(f"\n position of the end effector from inverse solution: {P[0][0].round(decimals=2)}i {P[1][0].round(decimals=2)}j {P[2][0].round(decimals=2)}k with respect to the base frame\n")

# verified for
# x, y, z= 1, 1, 1
# l1, l2, l3= 1, 1, 0.1


