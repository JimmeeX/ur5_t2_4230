#!/usr/bin/env python

"""
Handles Object Spawning

Responsibilities
/spawner/set_auto (Boolean - ON/OFF)
/spawner/create_object (String - Model Name)
"""

import rospy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, Pose, Quaternion

from gazebo_msgs.srv import (
    SpawnModel,
    SpawnModelRequest,
    SpawnModelResponse
)

import os
import random

from collections import Counter

# Object Combinations
COLORS = ['blue', 'green', 'orange', 'pink', 'red']
SHAPES = ['circle', 'triangle', 'square']
OBJECTS = [color + '_' + shape for shape in SHAPES for color in COLORS]

# ROS Internal Timer
SLEEP_RATE = 3 # Hz

# Spawn Interval
SPAWN_DELAY = 5.0 # seconds

# Spawn Location TODO: Change based on conveyor location
DEFAULT_LOC_X = 1.5
DEFAULT_LOC_Z = 0.05 / 2 # Half up so object doesn't phase through ground
DEFAULT_MIN_Y = 0.0
DEFAULT_MAX_Y = 1.0


# Generate Objects
class Spawner():
    def __init__(self, *args, **kwargs):
        print("Initialising Spawner Node")
        self._rate = rospy.Rate(SLEEP_RATE)

        # Initialise Variables
        self._is_auto = True
        self._object_counter = Counter()

        # Initialise Publishers & Subscribers
        rospy.Subscriber("/spawner/set_auto", Bool, self.handleSetAuto, queue_size=1)
        rospy.Subscriber("/spawner/create_object", String, self.handleCreateObject, queue_size=1)

        # Initialise Servers & Clients
        print("Waiting for gazebo/spawn_sdf_model service...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        print("Connected to gazebo server!")

        self._spawn_client = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        # Infinite Loop
        self.spawnLoop()

        return


    def spawnLoop(self):
        counter = 0

        # Exit program cleanly on ctrl+c (as opposed to 'while True')
        while not rospy.is_shutdown():
            if not self._is_auto: continue

            if counter >= SPAWN_DELAY * SLEEP_RATE:
                # Select Random Object
                model = OBJECTS[random.randint(0, len(OBJECTS)-1)]

                # Spawn Object & Reset Counter
                self.spawnObject(model)
                counter = 0

            counter += 1

            self._rate.sleep()

        return


    def handleSetAuto(self, msg):
        """
        msg format
            boolean - Set Auto Generate Objects
        """
        self._is_auto = msg.data


    def handleCreateObject(self, msg):
        """
        msg format
            string - Object Model Name (eg, "blue_circle")
        """

        self.spawnObject(msg.data)


    def spawnObject(self, model, pose=None):
        """
        Generates requested object in Gazebo Server
        """
        model_name = model + "_" + str(self._object_counter[model])

        file_path = os.environ["GAZEBO_MODEL_PATH"] + '/4230_objects/' + model + '.sdf'
        with open(file_path, 'r') as f:
            model_xml = f.read()

        if not pose:
            pose = Pose(
                Point(
                    x=DEFAULT_LOC_X,
                    y=random.uniform(DEFAULT_MIN_Y, DEFAULT_MAX_Y),
                    z=DEFAULT_LOC_Z
                ),
                Quaternion(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0
                )
            )

        request = SpawnModelRequest(
            model_name=model_name,
            model_xml=model_xml,
            robot_namespace=model_name,
            initial_pose=pose,
            reference_frame="world"
        )

        # Send Request && Process Response
        try:
            response = self._spawn_client(request)
            print(response)
            self._object_counter[model] += 1

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

        return True


if __name__ == "__main__":
    rospy.init_node('spawner')
    Spawner()
    rospy.spin()