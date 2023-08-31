import numpy as np
import math

def Stanford_end_effector(q1, q2, p3):
  # Base frame and joint 1
  H_01= np.array([[np.cos(q1), -np.sin(q1), 0, 0],
                  [np.sin(q1),  np.cos(q1), 0, 0],
                  [    0     ,      0     , 1, 0],
                  [    0     ,      0     , 0, 1]])
  
  # Joint 1 and joint 2
  H_12= np.array([[np.cos(q2), -np.sin(q2), 0, 0],
                  [    0     ,      0     , -1, 0],
                  [np.sin(q2),  np.cos(q2), 0, l1],
                  [    0     ,      0     , 0, 1]])
  
  # Joint 2 and joint 3
  H_23= np.array([[    1     ,      0     , 0, l2],
                  [    0     ,      1     , 0, 0],
                  [    0     ,      0     , 1, 0],
                  [    0     ,      0     , 0, 1]])
  
  p3 = np.transpose(p3)
  prod = np.matmul(np.matmul(np.matmul(H_01, H_12), H_23), p3)
  p0 = prod[:-1]
  return p0

q1 = 30 # Joint 1 angle
q1 = math.radians(q1)

q2 = 60 # Joint 2 angle
q2 = math.radians(q2)

d = 5 # Movement range of prismatic joint
p3 = [ 0, 0, d, 1]

# Length of links
l1 = 10
l2 = 10

p0 = Stanford_end_effector(q1, q2, p3)
print(p0)