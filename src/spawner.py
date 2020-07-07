#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, Pose, Quaternion

from gazebo_msgs.srv import (
    SpawnModel,
    SpawnModelRequest,
    SpawnModelResponse
)

class Spawner():
    def __init__(self, *args, **kwargs):
        print("Initialising Spawner Node")

        self._num_objects = 0

        print("Waiting for gazebo/spawn_urdf_model service...")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        print("Connected to gazebo server!")
        self._spawn_client = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

        self.spawnObject()
        return

    def spawnObject(self):
        # request = SpawnModelRequest()
        model_name = "object_" + str(self._num_objects)

        with open('../urdf/red_box.urdf', 'r') as f:
            model_xml = f.read()

        # print(model_xml)

        request = SpawnModelRequest(
            model_name=model_name,
            model_xml=model_xml,
            robot_namespace=model_name,
            initial_pose=Pose(
                Point(
                    x=2.0,
                    y=2.0,
                    z=0.2
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

        self._num_objects += 1
        try:
            response = self._spawn_client(request)
            print(response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return

    # def createObj(self):
    #     return


if __name__ == "__main__":
    rospy.init_node('spawner')
    Spawner()
    rospy.spin()