from sympy import zeros, symbols, Matrix, diff, pprint, simplify

def main():
    # Define symbolic variables
    num_joints = int(input('Enter the number of joints: '))
    joint_angles = symbols('q1:{}'.format(num_joints + 1))
    joint_velocities = symbols('dq1:{}'.format(num_joints + 1))
    joint_accelerations = symbols('ddq1:{}'.format(num_joints + 1))
    link_masses = symbols('m1:{}'.format(num_joints + 1))
    link_lengths = symbols('l1:{}'.format(num_joints + 1))
    gravitational_acceleration = symbols('g')
    inertial_terms = symbols('I1:{}'.format(num_joints + 1))

    # Define D(q) as a symbolic expression
    D_sys = symbols('d:{}{}'.format(num_joints, num_joints))

    # Take user input for the elements of the matrix D
    D_ele = [[symbols(f'd{i+1}{j+1}') for j in range(num_joints)] for i in range(num_joints)]
    D_mat = Matrix(D_ele)

    # Define specific values for joint angles and accelerations
    joint_angles_values = [1.1, 1.2, 1.4, 1.5, 1.7, 1.8]  # Replace with actual values
    joint_accelerations_values = [0.3, 0.2, 0.32, 0.25, 0.21, 0.25]  # Replace with actual values
    joint_angles_values = joint_angles_values[0:num_joints]
    joint_accelerations_values = joint_accelerations_values[0:num_joints]
    print("\n joint_angles_values = [1.1, 1.2, 1.4, 1.5, 1.7, 1.8] \n joint_accelerations_values = [0.3, 0.2, 0.32, 0.25, 0.21, 0.25] \n only first n values will be considered based on number of joints")

    # Calculate D(q).qdd
    D_ddq = calculate_D_ddq(D_mat, joint_accelerations_values)

    # Input specific potential term expression V(q)
    V_expression = 65  # Replace with actual symbolic expression for V(q)

    # Calculate Φ_k
    phi_matrix = calculate_phi(V_expression, joint_angles)

    # Calculate Christoffel symbols C(q,qd).qd
    C_matrix = calculate_christoffel_symbols(D_mat, joint_angles, joint_velocities)

    # Calculate τ(k) for each joint
    joint_torques = calculate_joint_torques(D_ddq, C_matrix, phi_matrix)

    # Print the joint torques with numeric values
    for i, torque in enumerate(joint_torques):
        print(f'τau{i+1} =')
        pprint(simplify(torque))
        print()

def calculate_D_ddq(D, ddq):
    D_ddq = Matrix(zeros(D.rows, 1))
    for k in range(D.rows):
        D_ddq[k] = D.row(k).dot(ddq)
    return D_ddq

def calculate_phi(V, q):
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
    return D_ddq + C + phi

if __name__ == "__main__":
    main()
