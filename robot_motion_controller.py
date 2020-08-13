# MTRN4230 Robotics
# Group 6 Assignment
# Robot Motion Module
#
# Authors: Samir Mustavi & Matthew Bourke
# Date: 27.07.2020
# Description: ROS module for providing actuation functions to the UR5 robot arm in the simulated Gazebo environment.
#              Desired x, y, z coordinates are received from the Image Processing node where this script calculates the
#              optimal joint angles.
#

import numpy as np
import cmath
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
#from std_msgs.msg import Header
#from trajectory_msgs.msg import JointTrajectory
#from trajectory_msgs.msg import JointTrajectoryPoint
#import rospy


a = np.array([0, -0.425, -0.39225, 0, 0, 0])
d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823], np.float)
alpha = np.array([pi / 2, 0, 0, pi / 2, -pi / 2, 0], np.float)


#  Helper function for returning transformation matrix of requested link frame
def calculate_link_t_matrix(n, th, c):
    a_m = a[n-1]
    d_m = d[n-1]
    alpha_m = alpha[n-1]
    theta_m = th[n-1, c]
    A_i = np.matrix([[cos(theta_m), -sin(theta_m)*cos(alpha_m), sin(theta_m)*sin(alpha_m), a_m*cos(theta_m)],
                     [sin(theta_m), cos(theta_m)*cos(alpha_m), -cos(theta_m)*sin(alpha_m), a_m*sin(theta_m)],
                     [0, sin(alpha_m), cos(alpha_m), d_m],
                     [0, 0, 0, 1]])
    return A_i


# Forward kinematics calculations
# Useful for generating desired end effector orientation matrix and validating inverse kinematics
def forward_kinematics(joint_angles):
    T_01 = calculate_link_t_matrix(1, joint_angles, 0)
    T_12 = calculate_link_t_matrix(2, joint_angles, 0)
    T_23 = calculate_link_t_matrix(3, joint_angles, 0)
    T_34 = calculate_link_t_matrix(4, joint_angles, 0)
    T_45 = calculate_link_t_matrix(5, joint_angles, 0)
    T_56 = calculate_link_t_matrix(6, joint_angles, 0)

    T_06 = T_01 * T_12 * T_23 * T_34 * T_45 * T_56
    return T_06


# Inverse kinematics solution
# Takes in x, y, z coordinates (in meters) and outputs desired joint angles (in degrees)
def inverse_kinematics(x, y, z):
    # Rotation matrix obtained from forward kinematics using ideal joint orientations
    T_06 = np.array([[-1, 0,  0, x],
                     [ 0, 1,  0, y],
                     [ 0, 0, -1, z],
                     [ 0, 0,  0, 1]])

    theta = np.matrix(np.zeros((6, 8)))
    p_05 = (T_06 * np.matrix([0, 0, -d[5], 1]).T - np.matrix([0, 0, 0, 1]).T)

    # Theta 1
    gamma = atan2(p_05[2 - 1, 0], p_05[1 - 1, 0])
    phi = acos(d[3] / sqrt(p_05[2 - 1, 0] * p_05[2 - 1, 0] + p_05[1 - 1, 0] * p_05[1 - 1, 0]))

    # The two solutions for theta 1 correspond to the shoulder being either left or right
    # 8 solutions exist in total
    # The first 4 consider left oriented shoulder, the last 4 consider right oriented shoulder
    theta[0, 0:4] = pi / 2 + gamma + phi
    theta[0, 4:8] = pi / 2 + gamma - phi
    theta = theta.real

    # Theta 5
    # Theta 5 determined if wrist orientation is 'up' or 'down'
    # config_offset defines solutions offsets based on theta 1 (first 4 solutions vs last 4 solutions)
    config_offset = [0, 4]
    for i in range(len(config_offset)):
        c = config_offset[i]
        T_10 = np.linalg.inv(calculate_link_t_matrix(1, theta, c))
        T_16 = T_10 * T_06

        # For each theta 1, there exists 2 potential solutions for theta 5
        theta[4, c:c+2] = + acos((T_16[2, 3] - d[3]) / d[5])
        theta[4, c+2:c+4] = - acos((T_16[2, 3] - d[3]) / d[5])
    theta = theta.real

    # Theta 6
    # Theta 6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.
    # config_offset redefined for solutions based on theta 1 and theta 5
    config_offset = [0, 2, 4, 6]
    for i in range(len(config_offset)):
        c = config_offset[i]
        T_10 = np.linalg.inv(calculate_link_t_matrix(1, theta, c))
        T_16 = np.linalg.inv(T_10 * T_06)

        # Based on theta 5, only one solutions exists for theta 6
        theta[5, c:c+2] = atan2((-T_16[1, 2] / sin(theta[4, c])), (T_16[0, 2] / sin(theta[4, c])))
    theta = theta.real

    # Theta 3
    # config_offset determined by theta 1 and theta 5 solutions
    config_offset = [0, 2, 4, 6]
    for i in range(0, len(config_offset)):
        c = config_offset[i]
        T_10 = np.linalg.inv(calculate_link_t_matrix(1, theta, c))
        T_65 = calculate_link_t_matrix(6, theta, c)
        T_54 = calculate_link_t_matrix(5, theta, c)
        T_14 = (T_10 * T_06) * np.linalg.inv(T_54 * T_65)
        P_13 = T_14 * np.matrix([0, -d[3], 0, 1]).T - np.matrix([0, 0, 0, 1]).T
        t3 = cmath.acos((np.linalg.norm(P_13)**2-a[1]**2-a[2]**2)/(2*a[1]*a[2]))

        # Two solutions exists, describing 'elbow up' or 'elbow down'
        theta[2, c] = t3.real
        theta[2, c+1] = -t3.real

    # Theta 2 & 4
    # config_offset redefined to present independent solutions for final joint angles theta 2 and theta 4
    config_offset = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(config_offset)):
        c = config_offset[i]
        T_10 = np.linalg.inv(calculate_link_t_matrix(1, theta, c))
        T_65 = np.linalg.inv(calculate_link_t_matrix(6, theta, c))
        T_54 = np.linalg.inv(calculate_link_t_matrix(5, theta, c))
        T_14 = (T_10 * T_06) * T_65 * T_54
        P_13 = T_14 * np.matrix([0, -d[3], 0, 1]).T - np.matrix([0, 0, 0, 1]).T

        # Theta 2
        # Theta 2 has only one solution, depending on elbow orientation
        theta[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a[2] * sin(theta[2, c]) / np.linalg.norm(P_13))
        # Theta 4
        T_32 = np.linalg.inv(calculate_link_t_matrix(3, theta, c))
        T_21 = np.linalg.inv(calculate_link_t_matrix(2, theta, c))
        T_34 = T_32 * T_21 * T_14
        # From all other defined joint angles, theta 4 only has one solution
        theta[3, c] = atan2(T_34[1, 0], T_34[0, 0])
    theta = theta.real

    # Due to multiple joint solutions, joint 2 is given joint rotation limit to avoid unwanted solutions
    joint2 = np.rad2deg(theta[1, :])
    size = np.size(joint2)
    min_value = 0
    index = []

    for i in range(size):
        # Joint 2 (shoulder joint) is chosen to be in the range -80 to 0 degrees
        # This corresponds to 'elbow up' orientation
        if 0 >= joint2[0, i] >= -80:
            element = i
            index.append(element)
    store_index = np.array(index)
    if len(store_index) == 0:
        min_value = np.min(np.abs(joint2))
    else:
        min_value = np.abs(joint2[0, store_index[0]])

    # filter possible solutions to a single solution that is the most ideal
    element = 0
    for index in store_index:
        if np.abs(joint2[0, index]) <= min_value:
            min_value = np.abs(joint2[0, index])
            element = index
    ideal_angles = np.rad2deg(theta[:, element])
    return ideal_angles


#def sub_echo(data):
#    # Callback function should maybe set a flag that enables robot movement to begin
#    rospy.loginfo("I heard %s", data.data)

def handle_get_coordinates(request):
    print("Received coordinates: " + request.x + ", " + request.y + ", " + request.z)
    angles = inverse_kinematics(request.x, request.y, request.z)
    return angles


if __name__ == "__main__":
    # Test values
    # Actual x, y, z will be received from image processing node
    x = -0.5
    y = 0.0
    z = 0.25

    rospy.init_node('robot_motion')
    server = rospy.Service('motion/move_to_object', ReceiveCoordinates, handle_get_coordinates)
    #server2 = rospy.Service('receive_coordinates', ReceiveCoordinates, handle_get_coordinates)
    #server3 = rospy.Service('receive_coordinates', ReceiveCoordinates, handle_get_coordinates)
    
    # Set up publisher and subscriber protocols
    #pub = rospy.Publisher('joint_waypoints', JointTrajectory, queue_size=10)
    # sub = rospy.Subscriber('image_processing', String, sub_echo)


    # Define a 'home' position for the robot to return to if no commands are received
    home_pos = np.matrix([[191.588279356832517], [-13.63151596868293], [-133.13413638498815],
                          [56.76565235367108],  [-90.00000000000000], [-78.41172064316748]])

    test_pos = np.matrix([[11.46], [22.91], [34.38], [45.84], [57.3], [68.75]])

    # Might have to initialise robot position before entering control loop

    # got_coordinates helps keep track of robot state. Once coordinates are received, robot actuation sequence begins
    got_coordinates = False

    # while not rospy.is_shutdown():
    #
    #     # Subscribe to image processing node and wait for x, y, z coordinates
    #     # When coordinates are received, set 'got_coordinates' to True
    #
    #     if got_coordinates:
    #         got_coordinates = False
    #         angles = inverse_kinematics(x, y, z)
    #
    #         # Publish joint angles to Gazebo node
    #
    #         # Once robot motion is complete, activate gripper
    #
    #         # Move end effector to box position
    #
    #         # Once robot motion is complete, deactivate gripper
    #
    #         # Return to 'home' position


    # Testing section
    angles = inverse_kinematics(x, y, z)
    print("Joint angles are:")
    for angle in angles:
        print("\t", angle[(0, 0)]*pi/180.0)
    H = forward_kinematics(angles * pi/180.0)
    print("\nEnd effector position calculated from FK is:")
    print("\tx = ", H[(0, 3)], "m\n\ty = ", H[(1, 3)], "m\n\tz = ", H[(2, 3)], "m")
    # print("\nError in position is:")
    # print("\tx_error = ", (H[(0, 3)]-x), "m\n\ty_error = ", (H[(1, 3)]-y), "m\n\tz_error = ", (H[(2, 3)]-z), "m")
