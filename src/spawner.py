#!/usr/bin/env python

"""
Handles Object & Container Spawning

Responsibilities
/spawner/set_auto (Boolean - ON/OFF)
/spawner/create_object (String - Model Name)
/spawner/create_container (Empty)

Author: James Lin
"""

import rospy

from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import Point, Pose, Quaternion

from gazebo_msgs.srv import (
    SpawnModel,
    SpawnModelRequest,
    SpawnModelResponse
)

import os
import random

from collections import Counter

# ROS Internal Timer
SLEEP_RATE = 3 # Hz

# Spawn Location set based on conveyor_in location
DEFAULT_OBJ_Y = 1.5
DEFAULT_OBJ_Z = 0.9 + 0.05 / 2 # Half up so object doesn't phase through ground
DEFAULT_OBJ_MIN_X = 0.35
DEFAULT_OBJ_MAX_X = 0.65

# Spawn Location of Container set based on conveyor_out location
DEFAULT_CONTAINER_X = -0.5
DEFAULT_CONTAINER_Y = 1
DEFAULT_CONTAINER_Z = 1


# Generate Objects
class Spawner():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[Spawner] Initialising Spawner Node")

        # Wait for Services to be ready
        rospy.loginfo("[Spawner] Waiting for gazebo/spawn_sdf_model service...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.loginfo("[Spawner] Connected to gazebo server!")


        # Initialise Variables
        # Grab Parameters from <rosparam> in the launch file
        self._is_auto = rospy.get_param("spawner/spawn_auto")
        self._spawn_delay = rospy.get_param("spawner/spawn_delay")
        self._colors = rospy.get_param("spawner/object_colors")
        self._shapes = rospy.get_param("spawner/object_shapes")

        self._objects = [color + '_' + shape for shape in self._shapes for color in self._colors]
        self._object_counter = Counter()
        self._container_counter = 0
        self._rate = rospy.Rate(SLEEP_RATE)


        # Initialise Subscribers
        self._subscribers = {}
        self._subscribers['spawner_set_auto'] = rospy.Subscriber("/spawner/set_auto", Bool, self.handleSetAuto, queue_size=1)
        self._subscribers['spawner_create_object'] = rospy.Subscriber("/spawner/create_object", String, self.handleCreateObject, queue_size=1)
        self._subscribers['spawner_create_container'] = rospy.Subscriber("/spawner/create_container", Empty, self.handleCreateContainer, queue_size=1)


        # Initialise Clients
        self._clients = {}
        self._clients['gazebo_spawn_sdf_model'] = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        # Infinite Loop
        self.spawnLoop()

        return


    def spawnLoop(self):
        counter = 0

        # Exit program cleanly on ctrl+c (as opposed to 'while True')
        while not rospy.is_shutdown():
            if not self._is_auto: continue

            if counter >= self._spawn_delay * SLEEP_RATE:
                # Select Random Object
                model = self._objects[random.randint(0, len(self._objects)-1)]

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


    def handleCreateContainer(self, msg):
        """
        msg format
            empty
        """
        self.spawnContainer()


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
                    x=random.uniform(DEFAULT_OBJ_MIN_X, DEFAULT_OBJ_MAX_X),
                    y=DEFAULT_OBJ_Y,
                    z=DEFAULT_OBJ_Z
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
            client = self._clients['gazebo_spawn_sdf_model']
            response = client(request)
            rospy.loginfo('[Spawner] Successfully spawned ' + model_name)
            self._object_counter[model] += 1

        except rospy.ServiceException as exc:
            rospy.logerr("[Spawner] Service did not process request: " + str(exc))
            return False

        return True


    def spawnContainer(self, pose=None):
        """
        Generates a container in Gazebo Server
        """
        model_name = "container_" + str(self._container_counter)

        file_path = os.environ["GAZEBO_MODEL_PATH"] + '/4230_objects/container.sdf'
        with open(file_path, 'r') as f:
            model_xml = f.read()

        if not pose:
            pose = Pose(
                Point(
                    x=DEFAULT_CONTAINER_X,
                    y=DEFAULT_CONTAINER_Y,
                    z=DEFAULT_CONTAINER_Z
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
            client = self._clients['gazebo_spawn_sdf_model']
            response = client(request)
            rospy.loginfo('[Spawner] Successfully spawned ' + model_name)
            self._container_counter += 1

        except rospy.ServiceException as exc:
            rospy.logerr("[Spawner] Service did not process request: " + str(exc))
            return False

        return True



if __name__ == "__main__":
    rospy.init_node('spawner')
    Spawner()
    rospy.spin()
