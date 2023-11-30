from sympy import zeros, symbols, Matrix, diff, pprint, simplify

def calculate_inertia_product(D, ddq):
    # Compute the product of the inertia matrix D and joint accelerations
    D_ddq = Matrix(zeros(D.rows, 1))
    for k in range(D.rows):
        D_ddq[k] = D.row(k).dot(ddq)
    return D_ddq

def calculate_potential_derivative(V, q):
    # Compute the partial derivatives of potential energy with respect to joint positions
    phi = Matrix([diff(V, q_k) for q_k in q])
    return phi

def calculate_christoffel_symbols(D, q, dq):
    C = Matrix(zeros(D.rows, 1))
    for k in range(D.rows):
        cijk = 0
        for i in range(D.rows):
            for j in range(D.rows):
                cijk += 0.5 * (diff(D[k, j], q[i]) + diff(D[i, k], q[j]) - diff(D[i, j], q[k])) * dq[i] * dq[j]
        C[k] = cijk
    return C

def calculate_joint_torques(D_ddq, C, phi):
    # Compute joint torques using the equation of motion
    return D_ddq + C + phi

def main():
    # Define symbolic variables for the robot dynamics

    # User input for the number of joints
    num_joints = int(input('Enter the number of joints: '))

    
    # Symbolic variables for joint positions, velocities, and accelerations
    q = symbols('q1:{}'.format(num_joints + 1))  # Joint angles
    dq = symbols('dq1:{}'.format(num_joints + 1))  # Joint velocities
    ddq = symbols('ddq1:{}'.format(num_joints + 1))  # Joint accelerations

    
    # Symbolic variables for other parameters
    m = symbols('m1:{}'.format(num_joints + 1))  # Link masses
    l = symbols('l1:{}'.format(num_joints + 1))  # Link lengths
    g = symbols('g')  # Gravitational acceleration
    I = symbols('I1:{}'.format(num_joints + 1))  # Inertial terms

    # Define the inertia matrix D(q) as a symbolic expression
    D = symbols('d:{}{}'.format(num_joints, num_joints))

    # Take user input for the elements of the matrix D
    D_elements = [[symbols(f'd{i+1}{j+1}') for j in range(num_joints)] for i in range(num_joints)]
    D_matrix = Matrix(D_elements)

    # Define specific values for joint angles and accelerations
    joint_angles_values = [30, 45, 60, 90, 120, 150]
    joint_accelerations_values = [1.5, 2.0, 1.2, 1.8, 1.0, 2.5] 


    # Consider only the first n values based on the number of joints
    joint_angles_values = joint_angles_values[0:num_joints]
    joint_accelerations_values = joint_accelerations_values[0:num_joints]

    print("\nJoint Angles Values: ", joint_angles_values)
    print("Joint Accelerations Values: ", joint_accelerations_values)
    print("Only first n values will be considered based on the number of joints")

    # Calculate D(q).qdd
    D_ddq = calculate_inertia_product(D_matrix, joint_accelerations_values)


   # Input specific potential term expression V(q)
    V_expression = 50  #potential_energy_expression & Replace with an actual symbolic expression for V(q)
 
    # Calculate Φ_k
    phi_matrix = calculate_potential_derivative(V_expression, q)

    # Calculate Christoffel symbols C(q,qd).qd
    C_matrix = calculate_christoffel_symbols(D_matrix, q, dq)

    # Calculate τ(k) for each joint
    joint_torques = calculate_joint_torques(D_ddq, C_matrix, phi_matrix)

    # Print the joint torques with numeric values
    for i, torque in enumerate(joint_torques):
        print(f'\nTorque on Joint {i+1} (τ_{i+1}):')
        pprint(simplify(torque))
    print()

if __name__ == "__main__":
    main()
