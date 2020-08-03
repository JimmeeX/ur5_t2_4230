#!/usr/bin/env python

"""
Main Flow Manager
Manages Orders on Server-side

Author: James Lin

"""

import rospy

from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import Point, Pose, Quaternion
from ur5_t2_4230.msg import (
    ConveyorBeltState,
    Order
)


from ur5_t2_4230.srv import (
    ConveyorBeltControl,
    ConveyorBeltControlRequest,
    ConveyorBeltControlResponse,
    OrderAdd,
    OrderAddRequest,
    OrderAddResponse,
    OrderDelete,
    OrderDeleteRequest,
    OrderDeleteResponse,
    OrderGet,
    OrderGetRequest,
    OrderGetResponse
)

from std_srvs.srv import (
    Trigger,
    TriggerRequest,
    TriggerResponse,
)

from utils.Orders import (
    Order,
    OrderList
)


# ROS Internal Timer
SLEEP_RATE = 3 # Hz

class OrderManager():
    def __init__(self, *args, **kwargs):
        rospy.loginfo("[OrderManager] Initialising Order Manager Node")

        # Wait for Services to be ready
        rospy.loginfo("[OrderManager] Waiting to connect with conveyor belts service...")
        rospy.wait_for_service("/ur5_t2_4230/conveyor/control/in")
        rospy.wait_for_service("/ur5_t2_4230/conveyor/control/out")
        rospy.loginfo("[OrderManager] Successfully connected to conveyor belts!")


        # Initialise Variables
        # Grab Parameters from <rosparam> in the launch file
        self._conveyor_power = rospy.get_param("order_manager/conveyor_power")
        self._colors = rospy.get_param("order_manager/object_colors")
        self._shapes = rospy.get_param("order_manager/object_shapes")

        self._order_list = OrderList() # Initialise with no current orders
        self._rate = rospy.Rate(SLEEP_RATE)


        # Initialise Subscribers
        self._subscribers = {}
        self._subscribers['msg_computer_vision'] = rospy.Subscriber("/msg/computer_vision", String, self.handleVisionCallback, queue_size=1)
        self._subscribers['break_beam_in_change'] = rospy.Subscriber("/break_beam_in_sensor_change", Bool, self.handleProximityChangeCallback, ('in'))
        self._subscribers['break_beam_out_change'] = rospy.Subscriber("/break_beam_out_sensor_change", Bool, self.handleProximityChangeCallback, ('out'))


        # Initialise Publishers
        self._publishers = {}
        self._publishers['spawner_create_container'] = rospy.Publisher("/spawner/create_container", Empty, queue_size=1)
        self._publishers['spawner_set_auto'] = rospy.Publisher("/spawner/set_auto", Bool, queue_size=1)


        # Initialise Servers
        self._servers = {}
        self._servers['order_add'] = rospy.Service('order_manager/add', OrderAdd, self.handleOrderAddRequest)
        self._servers['order_delete'] = rospy.Service('order_manager/delete', OrderDelete, self.handleOrderDeleteRequest)
        self._servers['order_get'] = rospy.Service('order_manager/get', OrderGet, self.handleOrderGetRequest)


        # Initialise Clients
        self._clients = {}
        self._clients['conveyor_control_in'] = rospy.ServiceProxy("/ur5_t2_4230/conveyor/control/in", ConveyorBeltControl)
        self._clients['conveyor_control_out'] = rospy.ServiceProxy("/ur5_t2_4230/conveyor/control/out", ConveyorBeltControl)

        return


    """SERVER-SIDE HANDLERS"""
    def handleOrderAddRequest(self, request):
        # TODO
        rospy.loginfo(request)
        # rospy.loginfo('[OrderManager - handleOrderAddRequest] ', request)

        response = OrderAddResponse(
            success=True,
            message="Order added successfully"
        )
        return response


    def handleOrderDeleteRequest(self, request):
        # TODO
        rospy.loginfo('[OrderManager - handleOrderDeleteRequest] '+ str(request))


        response = OrderAddResponse(
            success=True,
            message="Order deleted successfully"
        )
        return response


    def handleOrderGetRequest(self, request):
        rospy.loginfo('[OrderManager - handleOrderGetRequest] ' + str(request))
        # TODO

        response = OrderAddResponse(
            success=True,
            message="Order retrieved successfully"
        )
        return response

    """
    CLASS SUBSCRIBER CALLBACKS
    """

    def handleVisionCallback(self, msg):
        # TODO
        rospy.loginfo('[OrderManager - handleVisionCallback]' + str(msg))

        # print(msg)
        # shape, x, y, z = self.parseVision(msg)
        return

    def handleProximityChangeCallback(self, msg, id):
        """
        If a container/object arrives to break beam, then we stop to process the new order
        """
        if not msg.data: return # Ignore objects leaving beams

        if id == 'in': self._publishers['spawner_set_auto'].publish(Bool(False))

        client = self._clients['conveyor_control_' + id]
        request = ConveyorBeltControlRequest(ConveyorBeltState(power=0.00))

        try:
            response = client(request)
            rospy.loginfo('[OrderManager] Successfully stopped conveyor_belt_' + id)
        except rospy.ServiceException as exc:
            rospy.logerr("[OrderManager] Service did not process request: " + str(exc))

        return


if __name__ == "__main__":
    rospy.init_node('order_manager')
    OrderManager()
    rospy.spin()
