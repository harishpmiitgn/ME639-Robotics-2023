from numpy import cos, sin, array, transpose, matmul
import math

l1 = 10
l2 = 10
l3 = 10
l4 = 10
l5 = 10
l6 = 10
l7 = 10
l8 = 10


def UR5e(theta1, theta2, theta3, theta4, theta5, theta6, p6):
    
  H_01=    array([[ cos(theta1), - sin(theta1), 0  ,   0],
                  [ sin(theta1),   cos(theta1), 0  ,   0],
                  [    0       ,      0       , 1  ,  l1],
                  [    0       ,      0       , 0  ,   1]])
  
  H_12=    array([[    0       ,      0       , 0  , -l2],
                  [ sin(theta2),   cos(theta2), 0  ,   0],
                  [-cos(theta2),   sin(theta2), 0  ,   0],
                  [    0       ,      0       , 0  ,   1]])

  H_23=    array([[-cos(theta3),   sin(theta3), 0  ,  l3],
                  [ sin(theta3),   cos(theta3), 0  ,   0],
                  [    0       ,      0       , 1  , -l4],
                  [    0       ,      0       , 0  ,   1]])
  
  H_34=    array([[-cos(theta4),   sin(theta4), 0  , -l5],
                  [ sin(theta4),   cos(theta4), 0  ,   0],
                  [    0       ,      0       , 1  ,   0],
                  [    0       ,      0       , 0  ,   1]])
  
  H_45=    array([[    0       ,      0       , 1  ,   0],
                  [ sin(theta5),   cos(theta5), 0  ,   0],
                  [-cos(theta5),   sin(theta5), 0  ,  l6],
                  [    0       ,      0       , 0  ,   1]])

  H_56=    array([[    0       ,      0       ,-1  ,   0],
                  [ sin(theta6),   cos(theta6), 0  ,   0],
                  [-cos(theta6),  -sin(theta6), 0  ,  l7],
                  [    0       ,      0       , 0  ,   1]])  
  
  p6 = transpose(p6)
  prod =  matmul(matmul(matmul(matmul(matmul(matmul(H_01, H_12), H_23),H_34),H_45),H_56), p6)
  p0 = prod[:-1]
  return p0

theta1 = math.radians(int(input("Input the value of angle 1: ")))
theta2 = math.radians(int(input("Input the value of angle 2: ")))
theta3 = math.radians(int(input("Input the value of angle 3: ")))
theta4 = math.radians(int(input("Input the value of angle 4: ")))
theta5 = math.radians(int(input("Input the value of angle 5: ")))
theta6 = math.radians(int(input("Input the value of angle 6: ")))

p6 = [0, 0, l8, 1]
p0 = UR5e(theta1, theta2,theta3, theta4, theta5, theta6, p6)
print(p0)
