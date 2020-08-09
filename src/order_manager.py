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
    MoveToObject,
    MoveToObjectRequest,
    MoveToObjectResponse,
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
        self._clients['vision_detect_object'] = rospy.ServiceProxy("/vision/detect_object", Trigger)
        self._clients['motion_move_to_home'] = rospy.ServiceProxy("/motion/move_to_home", Trigger)
        self._clients['motion_move_to_object'] = rospy.ServiceProxy("/motion/move_to_object", MoveToObject)
        self._clients['motion_pickup_object'] = rospy.ServiceProxy("/motion/pickup_object", Trigger)
        self._clients['motion_move_to_container'] = rospy.ServiceProxy("/motion/move_to_container", Trigger)

        return


    """
    ####################
    SERVER-SIDE HANDLERS
    ####################
    """
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
    #####################
    CLIENT-SIDE FUNCTIONS
    #####################
    """

    def sendConveyorControlRequest(self, id, state):
        """
        Handles Turning both Conveyor On/Off by sending requests to
            - /ur5_t2_4230/conveyor/control/in
            - /ur5_t2_4230/conveyor/control/out

        Returns ConveyorBeltControlResponse object if exist otherwise None
        """

        service_name = 'conveyor_control_' + id
        client = self._clients[service_name]

        if state == 'on': power = self._conveyor_power # Default Conveyor Power
        elif state == 'off': power = 0.0 # Turn off Conveyor

        request = ConveyorBeltControlRequest(ConveyorBeltState(power=power))
        try:
            response = client(request)
            if response.success: rospy.loginfo('[OrderManager] Successfully turn ' + state + ' ' + service_name)
            else: rospy.logerr('[OrderManager] Conveyor control failed. Please try again')
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    def sendTriggerRequest(self, service_name):
        """
        General Trigger Request for any trigger-based service

        Returns TriggerResponse object
        """
        client = self._clients[service_name]

        request = TriggerRequest()
        try:
            response = client(request)
            if response.success: rospy.loginfo('[OrderManager] - ' + service_name + ': ' + response.message)
            else: rospy.logerr('[OrderManager] - ' + service_name + ': ' + response.message)
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    def sendMoveToObjectRequest(self, x, y, z):
        """
        Send Custom Trigger for robot to move to object
        """
        service_name = '/motion/move_to_object'
        client = self._clients[service_name]

        request = MoveToObjectRequest(
            location=Point(
                x=x,
                y=y,
                z=z
            )
        )
        try:
            response = client(request)
            if response.success: rospy.loginfo('[OrderManager] - ' + service_name + ': ' + response.message)
            else: rospy.logerr('[OrderManager] - ' + service_name + ': ' + response.message)
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    """
    ##########################
    CLASS SUBSCRIBER CALLBACKS
    ##########################
    """

    def handleVisionCallback(self, msg):
        # TODO (also temporary if vision is converted to server-side)
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

        # Stop Conveyor
        response = self.sendConveyorControlRequest(id=id, state='off')

        return response



if __name__ == "__main__":
    rospy.init_node('order_manager')
    OrderManager()
    rospy.spin()
