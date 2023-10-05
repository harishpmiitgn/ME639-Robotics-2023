import numpy as np
import math

def SCARA_jacobian(l1, l2, q1, q2):
  J = np.array([[-l2*np.sin(q1+q2) - l1*np.sin(q1), -l2*np.sin(q1+q2), 0],
                [ l2*np.cos(q1+q2) + l1*np.cos(q1),  l2*np.cos(q1+q2), 0],
                [                0                ,         0        ,-1],
                [                0                ,         0        , 0],
                [                0                ,         0        , 0],
                [                1                ,         1        , 0]])
  
  return J

# Length of links
l1 = 10
l2 = 10

q1 = 30 # Joint 1 angle
q1 = math.radians(q1)

q2 = 30 # Joint 2 angle
q2 = math.radians(q2)

jacobian = SCARA_jacobian(l1, l2, q1, q2)
print(jacobian)