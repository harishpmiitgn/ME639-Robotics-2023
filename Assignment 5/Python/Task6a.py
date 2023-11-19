from sympy import zeros,symbols, Matrix, diff, pprint, lambdify, Eq, solve, dsolve,sin, cos
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np

# Define symbolic variables
n = int(input('Enter no. of joints = '))  # Number of joints or degrees of freedom
q = symbols('q1:{}'.format(n+1))  # q (joint angles)
qd = symbols('qd1:{}'.format(n+1))  # qdot (joint velocities)
qdd = symbols('qdd1:{}'.format(n+1))  # qddot (joint accelerations)
m = symbols('m1:{}'.format(n+1))  # link masses
l = symbols('l1:{}'.format(n+1)) # link lengths
g = symbols('g')  # gravitational acceleration
I = symbols('I1:{}'.format(n+1))  # inertial terms

# Define D(q) and V(q) as symbolic expressions
# Define symbolic variables
D_symbols = symbols('d:{}{}'.format(n, n))
V_symbols = symbols('V')

# Take user input for the elements of the matrix D
print("Enter D(q) terms")
D_elements = [[(input(f'Enter element for d{i+1}{j+1}: ')) for j in range(n)] for i in range(n)]
# Create the matrix D
D = Matrix(D_elements)
# Display the matrix D
# print("\nMatrix D(q):")
# pprint(D)
D1 = Matrix(zeros(n, 1))
for k in range(n):
    d=0
    for i in range(n):
        d = d + (D[i,k]*qdd[i])
    D1[k] = d
# print("\nMatrix D(q).qdd:")
# pprint(D1)

V = input('\nEnter potential term expression V(q) = ')
# potential terms
phi = Matrix(zeros(n, 1))
for k in range(n):
    phi[k] = diff(V, q[k])
# print("\nMatrix Φ_k:")
# pprint(phi)


# Christoffel symbols
C =  Matrix(zeros(n, 1))
for k in range(n):
    cijk = 0
    for i in range(n):
        for j in range(n):
            cijk = cijk +  (0.5 * (diff(D[k,j],q[i]) + diff(D[i,k],q[j]) - diff(D[i,j],q[k]))*qd[i]*qd[j])
    C[k] = cijk
# print("\nChristoffel symbol Matrix C(q,qd).qd:")
# pprint(C)

tau = Matrix(zeros(n, 1))
for k in range(n):
    tau[k] = D1[k] + C[k] + phi[k]


# Take user input for masses, lengths and moment of inertia for link
m_values = [float(input(f'Enter value for link mass m{i+1}: ')) for i in range(n)]
I_values = [float(input(f'Enter value for moment of inertia I{i+1}: ')) for i in range(n)]
l_values = [float(input(f'Enter value for link length l{i+1}: ')) for i in range(n)]
substitutions = {m[i]: m_values[i] for i in range(n)}
substitutions.update({I[i]: I_values[i] for i in range(n)})
substitutions.update({l[i]: l_values[i] for i in range(n)})
g_value = 9.83
substitutions[g] = g_value
tau_substituted = tau.subs(substitutions)

#print(tau_substituted)

for k in range(n):
    print(f'τ{k+1} = ')
    print(tau_substituted[k])
    print('\n')

equations = [Eq(tau_substituted[i], 0.001) for i in range(n)]
#print(equations)

# for 3DOF PUMA

# Enter no. of joints = 3
# Enter D(q) terms       
# Enter element for d11: ((m2*l2**2*cos(q2)**2)/4)+m3*(0.5*l3*cos(q2+q3)+l2*cos(q2))**2 +I1+I2+I3  
# Enter element for d12: 0
# Enter element for d13: 0
# Enter element for d21: 0
# Enter element for d22: (0.25*m2*l2**2*sin(q2)**2)+m3*(0.25*l3**2+l2**2+l2*l3*cos(q3))+I2+I3
# Enter element for d23: m3*(0.25*l3**2+0.5*l2*l3*cos(q3))+I3
# Enter element for d31: 0
# Enter element for d32: m3*(0.25*l3**2+0.5*l2*l3*cos(q3))+I3
# Enter element for d33: m3*0.25*l3**2+I3
# Enter potential term expression V(q) = 0.5*m1*l1*g+m2*g*(l1+0.5*l2*sin(q2))+m3*g*(l1+l2*sin(q2)+0.5*l3*sin(q3))


# After running code and putting matrix values we get three torque equations
# Let m1=m2=m3=0.5
# Let l1=l2=l3=0.15
# Let I1=I2=I3=0.001
# plugging in above values we get three torque equations as shown as eq1, eq2, eq3 where each torque value set to 0.0001

# Define symbolic variables
q1, q2, q3, qd1, qd2, qd3, qdd1, qdd2, qdd3 = symbols('q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3')
# Given equations
eq1 = Eq(tau_substituted[0], 0.0001)
eq2 = Eq(tau_substituted[1], 0.0001)
eq3 = Eq(tau_substituted[2],0.0001)
# Solve for qdd1 and qdd2
sol = solve((eq1, eq2, eq3), (qdd1, qdd2, qdd3))

# Convert symbolic expressions to numerical functions
qdd1_func = lambdify((q1, q2, q3, qd1, qd2, qd3), sol[qdd1])
qdd2_func = lambdify((q1, q2, q3, qd1, qd2, qd3), sol[qdd2])
qdd3_func = lambdify((q1, q2, q3, qd1, qd2, qd3), sol[qdd3])

# Define the system of first-order ODEs
def system(y, t):
    q1, q2, q3, qd1, qd2, qd3 = y
    qdd1_val = qdd1_func(q1, q2, q3, qd1, qd2, qd3)
    qdd2_val = qdd2_func(q1, q2, q3, qd1, qd2, qd3)
    qdd3_val = qdd3_func(q1, q2, q3, qd1, qd2, qd3)
    
    return [qd1, qd2, qd3, qdd1_val, qdd2_val, qdd3_val]

# Initial conditions and time points
initial_conditions = [0, 0, 0, 0, 0, 0]
t = np.linspace(0, 10, 100)  # Adjust the time span as needed

# Solve the ODEs
solution = odeint(system, initial_conditions, t)

# Extracting the results
q1_result, q2_result, q3_result, qd1_result, qd2_result, qd3_result = solution[:, 0], solution[:, 1], solution[:, 2], solution[:, 3], solution[:, 4], solution[:, 5]

# Calculate accelerations using the lambdified functions
qdd1_result = qdd1_func(q1_result, q2_result, q3_result, qd1_result, qd2_result, qd3_result)
qdd2_result = qdd2_func(q1_result, q2_result, q3_result, qd1_result, qd2_result, qd3_result)
qdd3_result = qdd3_func(q1_result, q2_result, q3_result, qd1_result, qd2_result, qd3_result)


# Plotting accelerations
plt.figure(figsize=(12, 8))

# Plotting qdd1_result
plt.subplot(3,2, 1)
plt.plot(t, qdd1_result, label='qdd1', color='blue')
plt.title('qdd1 vs Time')
plt.xlabel('Time')
plt.ylabel('qdd1')
plt.legend()

# Plotting qdd2_result
plt.subplot(3, 2, 2)
plt.plot(t, qdd2_result, label='qdd2', color='purple')
plt.title('qdd2 vs Time')
plt.xlabel('Time')
plt.ylabel('qdd2')
plt.legend()

# Plotting qdd3_result
plt.subplot(3, 2, 3)
plt.plot(t, qdd3_result, label='qdd3', color='brown')
plt.title('qdd3 vs Time')
plt.xlabel('Time')
plt.ylabel('qdd3')
plt.legend()

plt.show()