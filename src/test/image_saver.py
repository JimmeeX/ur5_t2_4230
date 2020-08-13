#!/usr/bin/env python

"""
Saves Images to /test (for debugging)

Author: James Lin
"""

import rospy

import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import os
import random
import time


# ROS Internal Timer
SLEEP_RATE = 3 # Hz

# Spawn Location set based on conveyor_in location
DEFAULT_OBJ_Y = 1.5
DEFAULT_OBJ_Z = 0.9 + 0.05 / 2 # Half up so object doesn't phase through ground
DEFAULT_OBJ_MIN_X = 0.3
DEFAULT_OBJ_MAX_X = 0.7

# Spawn Location of Container set based on conveyor_out location
DEFAULT_CONTAINER_X = -0.5
DEFAULT_CONTAINER_Y = 1
DEFAULT_CONTAINER_Z = 1


DIR_NAME = "images"

# Generate Objects
class ImageSaver():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[ImageSaver] Initialising Node")

        # Initialise Subscribers
        self._sub = rospy.Subscriber("/camera/color/image_raw", Image, self.handler, queue_size=1)

        #
        self._bridge = CvBridge()

        return


    def handler(self, msg):
        try:
            im = self._bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)


        fileName = str(int(time.time() * 1000)) + '.jpg'
        filePath = os.path.join(DIR_NAME, fileName)
        rospy.loginfo("[ImageSaver] Saving: " + filePath)

        im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        cv2.imwrite(filePath, im_bgr)







if __name__ == "__main__":
    rospy.init_node('image_saver')
    ImageSaver()
    rospy.spin()
