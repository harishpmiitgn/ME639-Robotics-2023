import numpy as np
import math

R = []

for i in range(3):          # row 
    a =[]
    for j in range(3):      # column 
        a.append(int(input()))
    R.append(a)


r3 = []    #for first three link

for p in range(3):          #  row 
    b =[]
    for q in range(3):      # column 
        b.append(int(input()))
    r3.append(a)


q = [0, 0, 0]    # angle for wrist


def transpose(A, B):
    for i in range(3):
        for j in range(3):
            B[i][j] = A[j][i]

#Rotation matrix of the wrist
r3_trans = r3[:][:]
r3_trans = transpose(r3, r3_trans)

r36  = np.dot(r3_trans, R)

# using inverse kinematics
if (r36[1][3] ==0 & r36[2][3] == 0):
    if(r36[3][3] == 1):
        q[1] = 0
        q[0] = 0
        q[2] = math.atan(r36[2][1], r36[1][1])
    elif(r36[3][3] == -1):
        q[1] = 3.14
        q[0] = 0
        q[2] = math.atan(-r36[2][1], -r36[2][2])

else:
    q[1] = math.atan(r36[3][3], math.sqrt(1 - r36[3][3]**2))
    q[0] = math.atan( r36[1][3], r36[2][3])
    q[2] = math.atan(-r36[3][1], r36[3][2])


print("q[0] = angle_4 \nq[1] = angle_5 \nq[2] = angle_6")
print(q)