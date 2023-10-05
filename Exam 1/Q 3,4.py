import numpy as np
import math

def SCARA_end_effector(q1, q2, q3, q4, q5, q6, p6):
  # Base frame and joint 1
  H_01= np.array([[np.cos(q1), -np.sin(q1), 0, 0],
                  [np.sin(q1),  np.cos(q1), 0, 0],
                  [    0     ,      0     , 1, l1],
                  [    0     ,      0     , 0, 1]])
  
  # Joint 1 and joint 2
  H_12= np.array([[    0     ,      0     , 1,-l2],
                  [np.cos(q2), -np.sin(q2), 0, 0],
                  [np.sin(q2),  np.cos(q2), 1, 0],
                  [    0     ,      0     , 0, 1]])
  
  # Joint 2 and joint 3
  H_23= np.array([[np.cos(q3), -np.sin(q3), 0, l3],
                  [-np.sin(q3), -np.cos(q3), 0, 0],
                  [    0     ,      0     ,-1,-l4],
                  [    0     ,      0     , 0, 1]])
  
    # Joint 3 and joint 4
  H_34= np.array([[np.cos(q4), -np.sin(q4), 0, l5],
                  [-np.sin(q4), -np.cos(q4), 0, 0],
                  [    0     ,      0     ,-1,  0],
                  [    0     ,      0     , 0, 1]])
  
    # Joint 4 and joint 5
  H_45= np.array([[np.cos(q5), -np.sin(q5), 0, 0],
                  [    0     ,      0     , 1, 0],
                  [-np.sin(q5), -np.cos(q5),0,l6],
                  [    0     ,      0     , 0, 1]])
  
    # Joint 5 and joint 6
  H_56= np.array([[np.cos(q6), -np.sin(q6), 0, 0],
                  [    0     ,      0     ,-1, 0],
                  [-np.sin(q6), -np.cos(q6),0,l7],
                  [    0     ,      0     , 0, 1]])
  
  p6 = np.transpose(p6)
  prod = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(H_01, H_12), H_23), H_34), H_45), H_56), p6)
  p0 = prod[:-1]
  return p0

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

p0 = SCARA_end_effector(q1, q2, q3, q4, q5, q6, p6)
print(p0)