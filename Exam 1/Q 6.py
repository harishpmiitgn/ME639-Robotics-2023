import numpy as np
import math

def Jacobian(q1, q2, q3, q4, q5, q6):
  # Base frame and joint 1
  R_01= np.array([[np.cos(q1), -np.sin(q1), 0],
                  [np.sin(q1),  np.cos(q1), 0],
                  [    0     ,      0     , 1]])
  
  d_01= np.array([0, 0, l1])
  d_01 = d_01.reshape((3, 1))


  
  # Joint 1 and joint 2
  R_12= np.array([[    0     ,      0     , 1],
                  [np.cos(q2), -np.sin(q2), 0],
                  [np.sin(q2),  np.cos(q2), 1]])
  
  d_12= np.array([-l2, 0, 0])
  d_12 = d_01.reshape((3, 1))
  
  # Joint 2 and joint 3
  R_23= np.array([[np.cos(q3), -np.sin(q3), 0],
                  [-np.sin(q3), -np.cos(q3), 0],
                  [    0     ,      0     ,-1]])
  
  d_23= np.array([l3, 0, -l4])
  d_23 = d_01.reshape((3, 1))
  
    # Joint 3 and joint 4
  R_34= np.array([[np.cos(q4), -np.sin(q4), 0],
                  [-np.sin(q4), -np.cos(q4), 0],
                  [    0     ,      0     ,-1]])
  
  d_34= np.array([l5, 0, 0])
  d_34 = d_01.reshape((3, 1))
  
    # Joint 4 and joint 5
  R_45= np.array([[np.cos(q5), -np.sin(q5), 0],
                  [    0     ,      0     , 1],
                  [-np.sin(q5), -np.cos(q5),0]])
  
  d_45= np.array([0, 0, l6])
  d_45 = d_01.reshape((3, 1))
  
    # Joint 5 and joint 6
  R_56= np.array([[np.cos(q6), -np.sin(q6), 0],
                  [    0     ,      0     ,-1],
                  [-np.sin(q6), -np.cos(q6),0]])
  
  d_56= np.array([0, 0, l7])
  d_56 = d_01.reshape((3, 1))

  #Calculating the O matrices
  O_0= np.array([0, 0, 0])
  O_0 = d_01.reshape((3, 1))

  O_1 = d_01 + O_0
  O_2 = np.matmul(R_01, d_12) + O_1
  O_3 = np.matmul(np.matmul(R_01, R_12), d_23) + O_2
  O_4 = np.matmul(np.matmul(np.matmul(R_01, R_12), R_23), d_34) + O_3
  O_5 = np.matmul(np.matmul(np.matmul(np.matmul(R_01, R_12), R_23), R_34), d_45) + O_4
  O_6 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(R_01, R_12), R_23), R_34), R_45), d_56) + O_5
  
  z_0 = z_1 = z_2 = z_3 = z_4 = z_5 = np.array([0, 0, 1]).reshape((3,1))

  j_1 = np.array([-O_6[1]+O_0[1], O_6[0]-O_0[0], 0])
  j_1 = j_1.reshape((3,1))

  j_2 = np.array([-O_6[1]+O_1[1], O_6[0]-O_1[0], 0])
  j_2 = j_2.reshape((3,1))

  j_3 = np.array([-O_6[1]+O_2[1], O_6[0]-O_2[0], 0])
  j_3 = j_3.reshape((3,1))

  j_4 = np.array([-O_6[1]+O_3[1], O_6[0]-O_3[0], 0])
  j_4 = j_4.reshape((3,1))

  j_5 = np.array([-O_6[1]+O_4[1], O_6[0]-O_4[0], 0])
  j_5 = j_5.reshape((3,1))

  j_6 = np.array([-O_6[1]+O_5[1], O_6[0]-O_5[0], 0])
  j_6 = j_6.reshape((3,1))

  j = np.bmat([[j_1, j_2, j_3, j_4, j_5, j_6], [z_0, z_1, z_2, z_3, z_4, z_5]])
  return j

q1 = 30 # Joint 1 angle
q1 = math.radians(q1)

q2 = 30 # Joint 2 angle
q2 = math.radians(q2)

q3 = 30 # Joint 3 angle
q3 = math.radians(q3)

q4 = 30 # Joint 4 angle
q4 = math.radians(q4)

q5 = 30 # Joint 5 angle
q5 = math.radians(q5)

q6 = 30 # Joint 6 angle
q6 = math.radians(q6)

l8 = 5 # Distance till end effector
p6 = [0, 0, l8, 1]

l1 = 10
l2 = 10
l3 = 10
l4 = 10
l5 = 10
l6 = 10
l7 = 10
l8 = 10

J = Jacobian(q1, q2, q3, q4, q5, q6)
print(J)