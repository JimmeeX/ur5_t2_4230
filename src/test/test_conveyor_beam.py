#!/usr/bin/env python

"""
Script to test that the input of break beams are read correctly, and the conveyor controls are functionining correctly.

This program will stop the respective conveyors for 5 seconds before resuming whenever its respective break beam detects sth.

Author: James Lin
Date: 29/07/2020
"""

import rospy
import argparse

import threading
import time

from ur5_t2_4230.srv import (
    ConveyorBeltControl,
    ConveyorBeltControlRequest,
    ConveyorBeltControlResponse
)

from std_msgs.msg import Bool, Empty
from ur5_t2_4230.msg import Proximity, ConveyorBeltState

SLEEP_RATE = 3 # Hz
CONVEYOR_COOLDOWN = 5 # 5 seconds
CONVEYOR_POWER = 25.00 # 25% power

class TestConveyorBeam():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("Initialising TestConveyorBeam Node")
        self._rate = rospy.Rate(SLEEP_RATE)

        # Initialise Subscribers
        self._bb_in_sub = rospy.Subscriber("/break_beam_in_sensor_change", Bool, self.handleProximityChange, ('in'))
        self._bb_out_sub = rospy.Subscriber("/break_beam_out_sensor_change", Bool, self.handleProximityChange, ('out'))

        # Initialise Publishers
        self._spawn_container_pub = rospy.Publisher("/spawner/create_container", Empty, queue_size=1)
        self._spawn_set_auto_pub = rospy.Publisher("/spawner/set_auto", Bool, queue_size=1)

        # Initialise Client Server Handlers
        rospy.loginfo("Waiting to connect with conveyor belts service...")
        rospy.wait_for_service("/ur5_t2_4230/conveyor/control/in")
        rospy.wait_for_service("/ur5_t2_4230/conveyor/control/out")
        rospy.loginfo("Successfully connected to conveyor belts!")

        self._cc_in_client = rospy.ServiceProxy("/ur5_t2_4230/conveyor/control/in", ConveyorBeltControl)
        self._cc_out_client = rospy.ServiceProxy("/ur5_t2_4230/conveyor/control/out", ConveyorBeltControl)

        return


    def handleProximityChange(self, msg, id):
        rospy.logdebug('[handleProximityChange - ' + id + '] msg: ' + str(msg))

        # Incoming Objects
        if not msg.data: return True

        # Stop Conveyor
        request = ConveyorBeltControlRequest(ConveyorBeltState(power=0.00))

        if id == 'in':
            # Pickable objects

            # Stop Spawning Temporarily
            self._spawn_set_auto_pub.publish(Bool(False))
            client = self._cc_in_client

        else:
            # id == 'out'; Container Objects
            client = self._cc_out_client
        


        try:
            response = client(request)
            rospy.loginfo('Successfully stopped conveyor_belt_' + id)

            self._cooldown_thread = threading.Thread(name='CooldownThread_' + id, target=self.cooldownThread, args=(client, id))
            self._cooldown_thread.start()

        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

        return True

    
    def cooldownThread(self, client, id):
        # Start Conveyor again after cooldown
        rospy.logdebug('[CooldownThread_]' + id)
        duration = 0
        while duration < CONVEYOR_COOLDOWN:
            rospy.logdebug('[CooldownThread_' + id + ']: ' + str(duration))
            duration += 1
            time.sleep(1)

        # Start Conveyor again
        if id == 'in':
            # Resume spawning objects
            self._spawn_set_auto_pub.publish(Bool(True))
        else:
            # Spawn the next container
            self._spawn_container_pub.publish(Empty())

        request = ConveyorBeltControlRequest(ConveyorBeltState(power=CONVEYOR_POWER))

        try:
            response = client(request)
            rospy.loginfo('Successfully started conveyor_belt_' + id)

        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        
        return



if __name__ == "__main__":
    rospy.init_node('test_conveyor_beam')
    TestConveyorBeam()
    rospy.spin()
    