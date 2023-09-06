# Name: Rhitosparsha Baishya
# Roll No.: 23310039
# ME 639 Exam 1

import numpy as np

th1=30
th2=60
th3=90
th4=45
th5=50
th6=80

d1,r2,r3,d5,d4,d6=1,5,4,2,2,1

R1 =([np.cos(th1),np.sin(th1),0],
            [np.sin(th1),np.cos(th1),0],
            [0,0,1])

R2 =([np.cos(th2),np.sin(th2),0],
            [np.sin(th2),np.cos(th2),0],
            [0,0,1])

R3 =([np.cos(th3),np.sin(th3),0],
            [np.sin(th3),np.cos(th3),0],
            [0,0,1])

R4 =([np.cos(th4),np.sin(th4),0],
            [np.sin(th4),np.cos(th4),0],
            [0,0,1])

R5 =([np.cos(th5),np.sin(th5),0],
            [np.sin(th5),np.cos(th5),0],
            [0,0,1])

R6 =([np.cos(th6),np.sin(th6),0],
            [np.sin(th6),np.cos(th6),0],
            [0,0,1])



R0_1 = np.array([[1, 0, 0],
                 [0, 1, 0],
                 [0, 0, 1]])

d0_1 = np.array([[0],
                 [0],
                 [d1]])

R1_2a = np.array([[1, 0, 0],
                 [0, 0, -1],
                 [0, -1, 0]])

R1_2b = np.array([[0, -1, 0],
                 [1, 0, 0],
                 [0, 0, 1]])

R1_2 = np.dot(R1_2a,R1_2b)

d1_2 = np.array([[0],
                 [0],
                 [0]])

R2_3 = np.array([[1, 0, 0],
                 [0, 1, 0],
                 [0, 0, 1]])

d2_3 = np.array([[r2],
                 [0],
                 [0]])

R3_4 = np.array([[0, 1, 0],
                 [-1, 0,  0],
                 [0, 0, 1]])

d3_4 = np.array([[0],
                 [r3],
                 [d4]])

R4_5 = np.array([[1, 0, 0],
                 [0, 0,  1],
                 [0, -1, 0]])

d4_5 = np.array([[0],
                 [0],
                 [d5]])

R5_6 = np.array([[1, 0, 0],
                 [0, 0,  -1],
                 [0, 1, 0]])

d5_6 = np.array([[0],
                 [0],
                 [d6]])

H1=np.dot(R0_1,R1)
H2=np.dot(R1_2,R2)
H3=np.dot(R2_3,R3)
H4=np.dot(R3_4,R4)
H5=np.dot(R4_5,R5)
H6=np.dot(R5_6,R6)

print(H1)
print(H2)
print(H3)
print(H4)
print(H5)
print(H6)


