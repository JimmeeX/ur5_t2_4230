#!/usr/bin/env python

"""
Vision Module
Responsible for object detection. Translation from MATLAB Code

Author: James Lin
Credits: Rowena Dai, Jason Quek
"""

from geometry_msgs.msg import Point

from sensor_msgs.msg import (
    Image
)

from ur5_t2_4230.srv import (
    ObjectDetect,
    ObjectDetectRequest,
    ObjectDetectResponse
)

import rospy
import cv2
import time
import os

from cv_bridge import CvBridge, CvBridgeError

from utils.detect_object import detectObject

SLEEP_RATE = 3 # Hz

DIR_NAME = "images"

class Vision():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[Vision] Initialising Node")

        self._rate = rospy.Rate(SLEEP_RATE)
        self._bridge = CvBridge()
        self._im = None # Latest Image

        # Initialise Subscribers
        self._subscribers = {}
        self._subscribers['camera_color_image_raw'] = rospy.Subscriber('/camera/color/image_raw', Image, self.handleImageCallback, queue_size=1)

        # Initialise Publishers
        self._publishers = {}
        self._publishers['vision_sent_image'] = rospy.Publisher('/vision/sent_image', Image, queue_size=1)

        # Initialise Servers
        self._servers = {}
        self._servers['vision_detect_object'] = rospy.Service("/vision/detect_object", ObjectDetect, self.handleDetectObject)


    """
    ####################
    SERVER-SIDE HANDLERS
    ####################
    """
    def handleDetectObject(self, request):
        self.saveImage()
        color, shape, x, y, z = detectObject(self._im)

        response = ObjectDetectResponse(
            success=True,
            message='{} {} detected at [x={}, y={} z={}]'.format(color, shape, x, y, z),
            color=color,
            shape=shape,
            location=Point(
                x=x,
                y=y,
                z=z
            )
        )

        return response


    """
    ##########################
    CLASS SUBSCRIBER CALLBACKS
    ##########################
    """
    def handleImageCallback(self, msg):
        try:
            self._im = self._bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)


    """
    ################
    HELPER FUNCTIONS
    ################
    """
    def saveImage(self):
        fileName = str(int(time.time() * 1000)) + '.jpg'
        filePath = os.path.join(DIR_NAME, fileName)
        rospy.loginfo("[Vision] Saving: " + filePath)

        im_bgr = cv2.cvtColor(self._im, cv2.COLOR_RGB2BGR)
        cv2.imwrite(filePath, im_bgr)


if __name__ == '__main__':
    rospy.init_node('vision')
    Vision()
    rospy.spin()
