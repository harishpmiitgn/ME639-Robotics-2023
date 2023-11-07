import sympy as sym
import numpy as np
n = int(input()) #no. of dof
g = [] #dV/dqk matrix
V = sym.sympify(input()) #V(q) input
for i in range(n):
  g.append(sym.diff(V,sym.symbols("q"+str(i+1)))) #calculating values of g


d = [] #D(q)
for i in range(n):
  drow = []
  for j in range(n):
    drow.append(sym.sympify(input()))
  d.append(drow) #D(q) input


c = [] #christoffel's symbols of the first kind
for k in range(n):
  crow = []
  for i in range(n):
    for j in range(n):
      A = sym.diff(d[k][j],sym.symbols("q"+str(i+1)))
      B = sym.diff(d[k][i],sym.symbols("q"+str(j+1)))
      C = sym.diff(d[i][j],sym.symbols("q"+str(k+1)))
      crow.append(0.5*(A+B-C)) #calculation of cijk
  c.append(crow)
c


#creating new symbols for first and second order derivatives of q
qdubdot = []
qdot = []
for k in range(n):
  qdubdot.append(sym.symbols("q"+str(k+1)+"dubdot"))
  qdot.append(sym.symbols("q"+str(k+1)+"dot"))
#finding torque, i.e. dynamic equations
tau = []
for k in range(n):
  D = 0
  for i in range(n):
    D = D+d[k][i]*qdubdot[i]
  tau.append(D+g[k]) #tau = D(q)q"+ g(q)
for k in range(n):
  C = 0
  p = 0
  for i in range(n):
    for j in range(n):
      C = C + c[k][p]*qdot[i]*qdot[j]
      p = p+1
  tau[k] = tau[k]+C #tau = D(q)q" + C(q,q')q' + g(q)
tau #matrix for system's dynamic equations, where tau[k] = torque at a particular joint