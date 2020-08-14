#!/usr/bin/env python

"""
Motion & Robot Arm Controller
Responsible for any movement for the robot arm

Author: James Lin
Credits: Matt Bourke
"""

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import (
    Trigger,
    TriggerRequest,
    TriggerResponse,
)

from ur5_t2_4230.srv import (
    MoveToObject,
    MoveToObjectRequest,
    MoveToObjectResponse,
)

import rospy

from utils.kinematics import (
    inverse_kinematics
)

DURATION = 1.0
SLEEP_RATE = 3 # Hz

CONTAINER_X = 0.5
CONTAINER_Y = 0
CONTAINER_Z = 0.2 + 0.1 + 0.05 - 0.3 # Conveyor Height + Container Height + Spacing - Robot Height

INITIAL_WAIT = 3.0

class Motion():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[Motion] Initialising Node")

        self._rate = rospy.Rate(SLEEP_RATE)

        # Initialise Publishers
        self._publishers = {}
        self._publishers['arm_controller_command'] = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)


        # Initialise Servers
        self._servers = {}
        self._servers['motion_move_to_object'] = rospy.Service("/motion/move_to_object", MoveToObject, self.handleMotionMoveToObjectRequest)
        self._servers['motion_pickup_object'] = rospy.Service("/motion/pickup_object", Trigger, self.handlePickupObjectRequest)
        # self._servers['motion_drop_object'] = rospy.Service("/motion/drop_object", Trigger, self.handleMockTrigger)
        self._servers['motion_move_to_container'] = rospy.Service("/motion/move_to_container", Trigger, self.handleMovetoContainer)


        # Sleep for duration until move robot to home position
        # Without sleep, handleMovetoContainer will somtimes not run in Gazebo
        shouldMoveToHome = False
        counter = 0
        while not shouldMoveToHome and not rospy.is_shutdown():
            if counter >= INITIAL_WAIT * SLEEP_RATE: shouldMoveToHome = True
            counter += 1

            self._rate.sleep()
        
        self.handleMovetoContainer(None)

    
    def handleMotionMoveToObjectRequest(self, request):
        """Mock Response Demo wait 5 seconds"""
        point = request.location
        rospy.logwarn('[Motion] Moving Arm to Object - ' + str(point.x) + ', ' + str(point.y) + ', ' + str(point.z))

        # Inverse Kinematics on X,Y,Z --> Joint Positions
        q = inverse_kinematics(point.x, point.y, point.z)

        self.publishArmControllerCommand(q)

        response = MoveToObjectResponse(
            success=True,
            message="Robot move to object successfully"
        )

        return response

    def handlePickupObjectRequest(self, request):
        # TODO: Temporarily just stops for 3 seconds
        isObjectPickedUp = False
        counter = 0
        while not isObjectPickedUp and not rospy.is_shutdown():
            if counter >= 3.0 * SLEEP_RATE: isObjectPickedUp = True
            counter += 1

            self._rate.sleep()

        response = TriggerResponse(
            success=True,
            message="Object Picked up successfully"
        )
        return response


    def handleMovetoContainer(self, request):
        q = inverse_kinematics(CONTAINER_X, CONTAINER_Y, CONTAINER_Z)

        self.publishArmControllerCommand(q)

        response = TriggerResponse(
            success=True,
            message="Robot move to container successfully"
        )

        return response

    
    def publishArmControllerCommand(self, waypoints):
        
        # Create the topic message
        traj = JointTrajectory()
        traj.header = Header()
        # Joint names for UR5
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                            'wrist_3_joint']

        pts = JointTrajectoryPoint()
        traj.header.stamp = rospy.Time.now()
        pts.positions = waypoints
        pts.time_from_start = rospy.Duration(DURATION)

        rospy.loginfo('[Motion] - New Robot Joints Variables: ' + str(waypoints))

        traj.points = []
        traj.points.append(pts)

        self._publishers['arm_controller_command'].publish(traj)

        # Sleep for duration
        motionIsFinished = False
        counter = 0
        while not motionIsFinished and not rospy.is_shutdown():
            if counter >= DURATION * SLEEP_RATE: motionIsFinished = True
            counter += 1

            self._rate.sleep()

        return


if __name__ == '__main__':
    rospy.init_node('motion')
    Motion()
    rospy.spin()
