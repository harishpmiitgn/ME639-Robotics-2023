import numpy as np

def calculate_joint_velocities(jacobian, end_effector_linear_velocities, end_effector_angular_velocities):
    try:
        end_effector_velocities = np.concatenate((end_effector_linear_velocities, end_effector_angular_velocities))
        jacobian_inv = np.linalg.pinv(jacobian)
        joint_velocities = np.dot(jacobian_inv, end_effector_velocities)
        return joint_velocities
    except np.linalg.LinAlgError:
        return None

def main():
    try:
        num_links = int(input("Specify the number of links "))

        jacobian = np.empty((6, num_links))

        print("\nSpecify elements of the Jacobian (row x row)")
        for i in range(6):
            for j in range(num_links):
                jacobian[i][j] = float(input(f"Jacobian[{i}][{j}]: "))

        end_effector_linear_velocities = np.array([float(v) for v in input("\nSpecify end-effector linear velocities (eg: 1,2,4) (m/s): ").split(',')])
        end_effector_angular_velocities = np.array([float(v) for v in input("Specify end-effector angular velocities (eg: 1,2,4) (rad/s): ").split(',')])

        # Calculate joint velocities
        joint_velocities = calculate_joint_velocities(jacobian, end_effector_linear_velocities, end_effector_angular_velocities)

        if joint_velocities is not None:
            joint_linear_velocities, joint_angular_velocities = joint_velocities[:3], joint_velocities[3:]
            print("\nJoint Linear Velocities:", joint_linear_velocities)
            print("Joint Angular Velocities:", joint_angular_velocities)
        else:
            print("\nJacobian is singular. Cannot calculate joint velocities.")
    except ValueError:
        print("Invalid input. Please enter valid numeric values.")

if __name__ == "__main__":
    main()
