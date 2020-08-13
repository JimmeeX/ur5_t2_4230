#!/usr/bin/env python

# MTRN4230 Group 6 Assignment
# Checkpoint 2
# Simple robot motion


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import (
    Trigger,
    TriggerRequest,
    TriggerResponse,
)

import rospy

from utils.kinematics import (
    inverse_kinematics
)


waypoints = [[0.3445, -1.265, 1.496, 1.339, 1.571, 1.920],
             [4.586, -1.020, 1.7, 0.889, 1.571, -0.126],
             [3.3441, -0.2379, -2.324, 0.991, -1.5708, -1.369]]


class Motion():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[Motion] Initialising Node")

        # Initialise Publishers
        self._publishers = {}
        self._publishers['arm_controller_command'] = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)


        # Initialise Servers
        self._servers = {}
        self._servers['motion_move_to_home'] = rospy.Service("/motion/move_to_home", Trigger, self.handleMockTrigger)
        # self._servers['motion_pickup_object'] = rospy.Service("/motion/pickup_object", Trigger, self.handleMockTrigger)
        # self._servers['motion_drop_object'] = rospy.Service("/motion/drop_object", Trigger, self.handleMockTrigger)
        # self._servers['motion_move_to_container'] = rospy.Service("/motion/move_to_container", Trigger, self.handleMockTrigger)
    
    def handleMotionMoveToObjectRequest(self, request):
        """Mock Response Demo wait 5 seconds"""
        # print(request)
        point = request.location
        rospy.logwarn(point.x, point.y, point.z)

        # Inverse Kinematics on X,Y,Z --> Joint Positions
        q = inverse_kinematics(point.x, point.y, point.z)

        self.publishArmControllerCommand(q)

        # Publish Message

        # Inverse Kinematics on X,Y,Z --> Joint Positions

        # Send Joint Positions to Gazebo

        # motionIsFinished = False
        # counter = 0
        # while not motionIsFinished and not rospy.is_shutdown():
        #     if counter >= 5.0 * SLEEP_RATE: motionIsFinished = True
        #     counter += 1
        #     # Publish Feedback??

        #     self._rate.sleep()

        response = MoveToObjectResponse(
            success=True,
            message="Robot move to object successfully"
        )

        return response

    
    def publishArmControllerCommand(self, waypoints):
        traj = JointTrajectory()
        traj.header = Header()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_joint_1', 'wrist_joint_2', 'wrist_joint_3']
        
        traj.points = JointTrajectoryPoint(
            positions=list([waypoints]),
            time_from_start=rospy.Duration(2.0)
        )

        self._publishers['arm_controller_command'].publish(traj)

        return




# def send_movement_command():
#     rospy.init_node('send_joints')
#     pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
#     traj = JointTrajectory()
#     traj.header = Header()
#     joint.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                          'wrist_joint_1', 'wrist_joint_2', 'wrist_joint_3']
#     rate = rospy.Rate(0.5)
#     count = 0
#     pts = JointTrajectoryPoint()
#     traj.header.stamp = rospy.Time.now()

#     while not rospy.is_shutdown():
#         count += 1
#         pts.positions = waypoints[count%len(waypoints)]
#         pts.time_from_start = rospy.Duration(1.0)

#         traj.points = []
#         traj.points.append(pts)
#         pub.publish(traj)
#         rate.sleep()


if __name__ == '__main__':
    rospy.init_node('motion')
    Motion()
    rospy.spin()
