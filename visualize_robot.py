import sys
import matplotlib.pyplot as plt
import numpy as np

def plot_robot(joint_angles, end_effector_position, plot_title):
    link_lengths = [1.0, 1.0, 1.0]  # Example link lengths
    x = [0]
    y = [0]

    total_angle = 0
    for i, angle in enumerate(joint_angles):
        total_angle += angle
        x.append(x[i] + link_lengths[i] * np.cos(np.radians(total_angle)))
        y.append(y[i] + link_lengths[i] * np.sin(np.radians(total_angle)))

    plt.plot(x, y, '-o', label=plot_title)
    plt.plot(end_effector_position[0], end_effector_position[1], 'rx', label='End Effector')


def setup_plot():
    plt.figure()
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.grid(True)
    plt.axis('equal')

def display_plot():
    plt.legend()
    plt.show()

def forward_kinematics(joint_angles, end_effector_position):
    setup_plot()
    plot_robot(joint_angles, end_effector_position, 'Forward Kinematics')
    display_plot()

def inverse_kinematics(joint_angles, end_effector_position):
    setup_plot()
    # Split into two sets of angles
    joint_angles_1 = joint_angles[:3]
    joint_angles_2 = joint_angles[3:]
    # Plot first solution
    plot_robot(joint_angles_1, end_effector_position, 'Inverse Kinematics - Solution 1')
    # Plot second solution
    plot_robot(joint_angles_2, end_effector_position, 'Inverse Kinematics - Solution 2')
    display_plot()

def main():
    print("Incoming argument size: ", len(sys.argv))
    if not(len(sys.argv) == 6 or len(sys.argv) == 9):
        print("Usage: python visualize_robot.py <end_effector_x> <end_effector_y> <joint_angle_1> <joint_angle_2> <joint_angle_3>")
        
        sys.exit(1)
    if len(sys.argv) == 6:
        # Get the end effector position and joint angles from command line arguments
        end_effector_x = float(sys.argv[1])
        end_effector_y = float(sys.argv[2])
        joint_angles = [float(arg) for arg in sys.argv[3:6]]
        # Call the forward kinematics function with the provided end effector position and joint angles
        forward_kinematics(joint_angles, [end_effector_x, end_effector_y])
    elif len(sys.argv) == 9:
        # Get the end effector position and joint angles from command line arguments
        end_effector_x = float(sys.argv[1])
        end_effector_y = float(sys.argv[2])
        joint_angles = [float(arg) for arg in sys.argv[3:9]]
        # Call the inverse kinematics function with the provided end effector position and joint angles
        inverse_kinematics(joint_angles, [end_effector_x, end_effector_y])
    else:
        print("Invalid number of arguments")
        sys.exit(1)


if __name__ == '__main__':
    main()
