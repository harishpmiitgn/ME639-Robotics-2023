import numpy as np
import math

def End_Effector_Position(q1, q2, q3, q4, q5, q6, p6):

    q1 = math.radians(q1)
    q2 = math.radians(q2)
    q3 = math.radians(q3)
    q4 = math.radians(q4)
    q5 = math.radians(q5)
    q6 = math.radians(q6)
    p6 = np.transpose(p6)

    # Creating 6 numpy arrays corresponding to the 6 homogeneous transformations

    H56 = np.array([[np.cos(q6),-np.sin(q6),0,0],[0,0,1,d6],[-np.sin(q6),-np.cos(q6),0, 0 ],[0,0,0,1]])

    H45 = np.array([[np.cos(q5),-np.sin(q5),0,0],[0,0,-1,-d5],[np.sin(q5),np.cos(q5),0, 0 ],[0,0,0,1]])

    H34 = np.array([[1,0,0,a3],[0, np.cos(q4),-np.sin(q4),0],[0, np.sin(q4),np.cos(q4), d4],[0,0,0,1]])

    H23 = np.array([[np.cos(q3),-np.sin(q3),0,a2],[np.sin(q3),np.cos(q3),0, 0 ],[0,0,1,-d3],[0,0,0,1]])

    H12 = np.array([[np.cos(q2),-np.sin(q2),0,0],[0,0,-1,-d2],[np.sin(q2),np.cos(q2),0, 0 ],[0,0,0,1]])

    H01 = np.array([[np.cos(q1),-np.sin(q1),0,0],[np.sin(q1),np.cos(q1),0, 0 ],[0,0,1,d1],[0,0,0,1]])

    ans = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(H01,H12),H23),H34),H45),H56), p6)
    p0 = ans[:-1]
    return p0

#assign values to all the joint variables used in the above function.
d1 = 1
d2 = 1
d3 = 1
d4 = 1
d5 = 1
d6 = 1
dp = 1

q1 = 1
q2 = 1
q3 = 1
q4 = 1
q5 = 1
q6 = 1

a2 = 1
a3 = 1

p6 = [0,0,dp,1]

Final_ans = End_Effector_Position(q1, q2, q3, q4, q5, q6, p6)
print(Final_ans)
