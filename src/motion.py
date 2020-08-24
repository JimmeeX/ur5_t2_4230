#!/usr/bin/env python

"""
Motion & Robot Arm Controller
Responsible for any movement for the robot arm

Author: James Lin
Credits: Matt Bourke
"""

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur5_t2_4230.msg import VacuumGripperState

from std_srvs.srv import (
    Trigger,
    TriggerRequest,
    TriggerResponse,
)

from ur5_t2_4230.srv import (
    PickupObject,
    PickupObjectRequest,
    PickupObjectResponse,
    VacuumGripperControl,
    VacuumGripperControlRequest,
    VacuumGripperControlResponse
)

import rospy

from utils.kinematics import (
    inverse_kinematics
)

DURATION = 1.0
SLEEP_RATE = 3 # Hz

CONVEYOR_HEIGHT = 0.2
CONTAINER_HEIGHT = 0.1
OBJECT_HEIGHT = 0.05
ROBOT_HEIGHT = 0.3
SPACING = 0.05

CONTAINER_X = 0.5
CONTAINER_Y = 0
CONTAINER_Z = CONVEYOR_HEIGHT + CONTAINER_HEIGHT + OBJECT_HEIGHT + SPACING - ROBOT_HEIGHT

UP_SHIFT = 0.25

INITIAL_WAIT = 3.0

class Motion():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[Motion] Initialising Node")

        self._rate = rospy.Rate(SLEEP_RATE)

        self._is_object_attached = False

        # Initialise Subscribers
        self._subscribers = {}
        self._subscribers['gripper_state'] = rospy.Subscriber('/arm_controller/gripper/state', VacuumGripperState, self.handleVacuumStateCallback, queue_size=1)

        # Initialise Publishers
        self._publishers = {}
        self._publishers['arm_controller_command'] = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

        # Initialise Servers
        self._servers = {}
        self._servers['motion_pickup_object'] = rospy.Service("/motion/pickup_object", PickupObject, self.handlePickupObjectRequest)
        self._servers['motion_drop_object'] = rospy.Service("/motion/drop_object", Trigger, self.handleDropObjectRequest)

        # Initialise Clients
        self._clients = {}
        self._clients['gripper_control'] = rospy.ServiceProxy('/arm_controller/gripper/control', VacuumGripperControl)


        # Sleep for duration until move robot to container
        # Without sleep, handleMovetoContainer will somtimes not run in Gazebo
        shouldMoveToHome = False
        counter = 0
        while not shouldMoveToHome and not rospy.is_shutdown():
            if counter >= INITIAL_WAIT * SLEEP_RATE: shouldMoveToHome = True
            counter += 1

            self._rate.sleep()
        
        self.handleDropObjectRequest(None)


    """
    ####################
    SERVER-SIDE HANDLERS
    ####################
    """
    def handlePickupObjectRequest(self, request):
        """
        Motion Sequence
        1. Move above object (coordinates provided)
        2. Enable Gripper
        3. Move down to contact object
        4. Move back up (with object)
        """

        point = request.location
        rospy.loginfo('[Motion] Received coordinates of Object - ' + str(point.x) + ', ' + str(point.y) + ', ' + str(point.z))

        # 1. Move above Object
        # Inverse Kinematics on X,Y,Z --> Joint Positions
        rospy.loginfo("[Motion] Moving arm to above object")
        q1 = inverse_kinematics(point.x, point.y, point.z + UP_SHIFT)
        self.publishArmControllerCommand(q1)

        # 2. Enable Gripper
        response_pickup_object = self.sendGripperControlRequest(enable=True)
        if not (response_pickup_object and response_pickup_object.success):
            response = PickupObjectResponse(
                success=False,
                message="Robot Gripper failed to operate"
            )
            return response

        # 3.
        rospy.loginfo("[Motion] Moving arm down to pickup object")
        q2 = inverse_kinematics(point.x, point.y, point.z)
        self.publishArmControllerCommand(q2)

        # Wait until Confirmation that object has been picked up
        rospy.loginfo("[Motion] Waiting for object attachement")
        while not self._is_object_attached and not rospy.is_shutdown():
            self._rate.sleep()
        rospy.loginfo("[Motion] Object attached!")

        # 4.
        rospy.loginfo("[Motion] Moving arm back up With Object")
        q3 = inverse_kinematics(point.x, point.y, point.z + UP_SHIFT)
        self.publishArmControllerCommand(q3)

        # Return Response
        response = PickupObjectResponse(
            success=True,
            message="Robot picked up object successfully"
        )

        return response


    def handleDropObjectRequest(self, request):
        """
        Motion Sequence
        1. Move above container
        2. Disable Gripper
        """

        rospy.loginfo('[Motion] Received command to drop object')

        # 1. Move above container
        q = inverse_kinematics(CONTAINER_X, CONTAINER_Y, CONTAINER_Z)
        self.publishArmControllerCommand(q)

        # 2. Disable Gripper
        response_pickup_object = self.sendGripperControlRequest(enable=False)
        if not (response_pickup_object and response_pickup_object.success):
            response = PickupObjectResponse(
                success=False,
                message="Robot Gripper failed to operate"
            )
            return response

        # Return Response
        response = TriggerResponse(
            success=True,
            message="Robot dropped object successfully"
        )

        return response


    """
    ##########################
    CLASS SUBSCRIBER CALLBACKS
    ##########################
    """
    def handleVacuumStateCallback(self, msg):
        self._is_object_attached = msg.attached


    """
    ################
    HELPER FUNCTIONS
    ################
    """
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


    def sendGripperControlRequest(self, enable):
        """
        Handles Turning Robot Gripper On/Off

        Returns VacuumGripperControlResponse object if exist otherwise None
        """

        service_key = 'gripper_control'
        client = self._clients[service_key]

        request = VacuumGripperControlRequest(enable=enable)
        try:
            response = client(request)
            if response.success: rospy.loginfo('[Motion] Successfully set gripper ' + str(enable))
            else: rospy.logerr('[Motion] Conveyor control failed. Please try again')
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[Motion] Service did not process request: ' + str(exc))
            return None


if __name__ == '__main__':
    rospy.init_node('motion')
    Motion()
    rospy.spin()
