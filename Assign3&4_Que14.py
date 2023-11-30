import numpy as np

def calculate_joint_velocities(jacobian, end_effector_linear_velocities, end_effector_angular_velocities):
    try:
        # Combine linear and angular velocities into a single array
        end_effector_velocities = np.concatenate((end_effector_linear_velocities, end_effector_angular_velocities))

        # Calculate the pseudo-inverse of the Jacobian matrix
        jacobian_inv = np.linalg.pinv(jacobian)

        # Calculate joint velocities
        joint_velocities = np.dot(jacobian_inv, end_effector_velocities)

        return joint_velocities
    except np.linalg.LinAlgError:
        return None

def main():
    try:
        # Get the number of links from the user
        num_links = int(input("Enter the number of links: "))

        # Initialize the Jacobian matrix with appropriate dimensions
        jacobian = np.empty((6, num_links))

        # Get user input for the Jacobian elements
        print("\nEnter elements of the Jacobian matrix (row x row)")
        for i in range(6):
            for j in range(num_links):
                while True:
                    try:
                        jacobian[i][j] = float(input(f"Jacobian[{i}][{j}]: "))
                        break
                    except ValueError:
                        print("Invalid input. Please enter valid numeric values.")

        # Get user input for end-effector velocities
        end_effector_linear_velocities = np.array([float(v) for v in input("\nEnter end-effector linear velocities (m/s, eg: 1,2,4): ").split(',')])
        end_effector_angular_velocities = np.array([float(v) for v in input("Enter end-effector angular velocities (rad/s, eg: 1,2,4): ").split(',')])

        # Calculate joint velocities
        joint_velocities = calculate_joint_velocities(jacobian, end_effector_linear_velocities, end_effector_angular_velocities)

        if joint_velocities is not None:
            # Split joint velocities into linear and angular components
            joint_linear_velocities, joint_angular_velocities = joint_velocities[:3], joint_velocities[3:]

            # Print the results
            print("\nJoint Linear Velocities:", joint_linear_velocities)
            print("Joint Angular Velocities:", joint_angular_velocities)
        else:
            print("\nJacobian is singular. Cannot calculate joint velocities.")
    except ValueError:
        print("Invalid input. Please enter valid numeric values.")

if __name__ == "__main__":
    main()
