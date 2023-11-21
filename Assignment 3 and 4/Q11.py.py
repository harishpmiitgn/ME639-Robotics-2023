from numpy import zeros,symbols, Matrix, diff, pprint

# symbolic variables
n = int(input('number of joints = ')) 
q = symbols('q1:{}'.format(n+1))  
qd = symbols('qd1:{}'.format(n+1))  # joint velocities
qdd = symbols('qdd1:{}'.format(n+1))  # joint accelerations
m = symbols('m1:{}'.format(n+1))  # mass
l = symbols('l1:{}'.format(n+1)) # lengths
g = symbols('g')  # for gravitational effect 
I = symbols('I1:{}'.format(n+1))  # inertia



d_sys = symbols('d:{}{}'.format(n, n))   # Define symbolic variables
v_sys = symbols('V')   # Define symbolic variables

# elements of the matrix D
print("D(q) terms")
d_elements = [[(input(f'Enter element for d{i+1}{j+1}: ')) for j in range(n)] for i in range(n)]
D = Matrix(d_elements)
D1 = Matrix(zeros(n, 1))
for k in range(n):
    d=0
    for i in range(n):
        d = d + (D[i,k]*qdd[i])
    D1[k] = d


V = input('\nV(q) terms = ')

Phe = Matrix(zeros(n, 1))
for k in range(n):
    Phe[k] = diff(V, q[k])


C =  Matrix(zeros(n, 1))
for k in range(n):
    cijk = 0
    for i in range(n):
        for j in range(n):
            cijk = cijk +  (0.5 * (diff(D[k,j],q[i]) + diff(D[i,k],q[j]) - diff(D[i,j],q[k]))*qd[i]*qd[j])
    C[k] = cijk

tau = Matrix(zeros(n, 1))
for k in range(n):
    tau[k] = D1[k] + C[k] + Phe[k]


for k in range(n):
    print(f'τ{k+1} = ')
    pprint(tau[k])
    print('\n')




