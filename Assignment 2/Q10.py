import numpy as np
import math

def RRR_jacobian(l1, l2, l3, q1, q2, q3):
  J = np.array([[-l3*np.sin(q1+q2+q3) - l2*np.sin(q1+q2) - l1*np.sin(q1), -l3*np.sin(q1+q2+q3) - l2*np.sin(q1+q2), -l3*np.sin(q1+q2+q3)],
                [ l3*np.cos(q1+q2+q3) + l2*np.cos(q1+q2) + l1*np.sin(q1),  l3*np.cos(q1+q2+q3) + l2*np.cos(q1+q2),  l3*np.cos(q1+q2+q3)],
                [                           0                           ,                    0                   ,          0          ],
                [                           0                           ,                    0                   ,          0          ],
                [                           0                           ,                    0                   ,          0          ],
                [                           1                           ,                    1                   ,          1          ]])
  
  return J

# Length of links
l1 = 10
l2 = 10
l3 = 10

q1 = 30 # Joint 1 angle
q1 = math.radians(q1)

q2 = 30 # Joint 2 angle
q2 = math.radians(q2)

q3 = 30 # Joint 3 angle
q3 = math.radians(q3)

jacobian = RRR_jacobian(l1, l2, l3, q1, q2, q3)
print(jacobian)