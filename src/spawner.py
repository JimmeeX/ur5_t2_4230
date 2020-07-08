#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, Pose, Quaternion

from gazebo_msgs.srv import (
    SpawnModel,
    SpawnModelRequest,
    SpawnModelResponse
)

import os
import random

colors = ['blue', 'green', 'orange', 'pink', 'red']
shapes = ['circle', 'triangle', 'square']
objects = [color + '_' + shape for shape in shapes for color in colors]

# Generate Objects
class Spawner():
    def __init__(self, *args, **kwargs):
        print("Initialising Spawner Node")

        # Initialise Variables
        self._num_objects = 0

        # Initialise Publishers & Subscribers
        # rospy.Subscriber("/spawn/continue", Bool)
        # self.valve1_sub = rospy.Subscriber("/arduino/valve1", Bool, self.handle_valve, (1), queue_size=1) # Jar 1


        # Initialise Servers & Clients
        print("Waiting for gazebo/spawn_sdf_model service...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        print("Connected to gazebo server!")

        self._spawn_client = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        for obj in objects:
            self.spawnObject(obj)
        # self.spawnObject("blue_square")
        # self.spawnObject("blue_circle")
        return

    def spawnObject(self, model):
        # Generate Request Object
        model_name = "object_" + str(self._num_objects)

        file_path = os.environ["GAZEBO_MODEL_PATH"] + '/4230_objects/' + model + '.sdf'
        with open(file_path, 'r') as f:
            model_xml = f.read()

        request = SpawnModelRequest(
            model_name=model_name,
            model_xml=model_xml,
            robot_namespace=model_name,
            initial_pose=Pose(
                Point(
                    x=1.5,
                    # y=1.0,
                    y=random.random(),
                    z=0.025 # Half of z-size=0.05 (so it sits on the ground)
                ),
                Quaternion(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0
                )
            ),
            reference_frame="world"
        )

        # Send Request && Process Response

        try:
            response = self._spawn_client(request)
            print(response)
            self._num_objects += 1

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return


if __name__ == "__main__":
    rospy.init_node('spawner')
    Spawner()
    rospy.spin()