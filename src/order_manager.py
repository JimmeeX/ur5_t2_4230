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

from rosbridge_library.srv import (
    SendBytes,
    SendBytesRequest,
    SendBytesResponse
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
        self._is_container_ready = False # Boolean stating whether container is ready for pick-and-drop

        self._order_list = OrderList(
            valid_colors=self._colors + ['none'],
            valid_shapes=self._shapes + ['none']
        )
        self._rate = rospy.Rate(SLEEP_RATE)


        # Initialise Subscribers
        self._subscribers = {}
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

        # Temporary Testing
        # self._servers['vision_detect_object'] = rospy.Service("/vision/detect_object", SendBytes, self.handleVisionDetectObjectRequest)
        # self._servers['motion_move_to_object'] = rospy.Service("/motion/move_to_object", MoveToObject, self.handleMotionMoveToObjectRequest)
        # self._servers['motion_move_to_home'] = rospy.Service("/motion/move_to_home", Trigger, self.handleMockTrigger)
        # self._servers['motion_pickup_object'] = rospy.Service("/motion/pickup_object", Trigger, self.handleMockTrigger)
        self._servers['motion_drop_object'] = rospy.Service("/motion/drop_object", Trigger, self.handleMockTrigger)
        # self._servers['motion_move_to_container'] = rospy.Service("/motion/move_to_container", Trigger, self.handleMockTrigger)


        # Initialise Clients
        self._clients = {}
        self._clients['conveyor_control_in'] = rospy.ServiceProxy("/ur5_t2_4230/conveyor/control/in", ConveyorBeltControl)
        self._clients['conveyor_control_out'] = rospy.ServiceProxy("/ur5_t2_4230/conveyor/control/out", ConveyorBeltControl)
        self._clients['vision_detect_object'] = rospy.ServiceProxy("/vision/detect_object", SendBytes) # TODO: Temporary
        self._clients['motion_move_to_home'] = rospy.ServiceProxy("/motion/move_to_home", Trigger)
        self._clients['motion_move_to_object'] = rospy.ServiceProxy("/motion/move_to_object", MoveToObject)
        self._clients['motion_pickup_object'] = rospy.ServiceProxy("/motion/pickup_object", Trigger)
        self._clients['motion_drop_object'] = rospy.ServiceProxy("/motion/drop_object", Trigger)
        self._clients['motion_move_to_container'] = rospy.ServiceProxy("/motion/move_to_container", Trigger)

        # Move Robot to Home (Container)
        return


    """
    ####################
    SERVER-SIDE HANDLERS
    ####################
    """
    def handleOrderAddRequest(self, request):
        # Validate && Add Order to Order List
        success, message, trigger_container = self._order_list.add_order(color=request.color, shape=request.shape, goal=request.goal)
        if not success: rospy.logerr(message)

        if trigger_container: self.startConveyorOut(spawn=True)

        response = OrderAddResponse(
            success=success,
            message=message
        )
        return response


    def handleOrderDeleteRequest(self, request):
        success, message = self._order_list.delete_order(id=request.id)
        if not success: rospy.logerr(message)

        response = OrderDeleteResponse(
            success=success,
            message=message
        )
        return response


    def handleOrderGetRequest(self, request):
        response = OrderGetResponse(
            orders_queued=self._order_list.orders_queued,
            orders_doing=self._order_list.orders_doing,
            orders_done=self._order_list.orders_done,
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

        service_key = 'conveyor_control_' + id
        client = self._clients[service_key]

        if state == 'on': power = self._conveyor_power # Default Conveyor Power
        elif state == 'off': power = 0.0 # Turn off Conveyor

        request = ConveyorBeltControlRequest(ConveyorBeltState(power=power))
        try:
            response = client(request)
            # if response.success: rospy.loginfo('[OrderManager] Successfully turn ' + state + ' ' + service_key)
            # else: rospy.logerr('[OrderManager] Conveyor control failed. Please try again')
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    def sendTriggerRequest(self, service_key):
        """
        General Trigger Request for any trigger-based service

        Returns TriggerResponse object
        """
        client = self._clients[service_key]

        request = TriggerRequest()
        try:
            response = client(request)
            if response.success: rospy.loginfo('[OrderManager] - ' + service_key + ': ' + response.message)
            else: rospy.logerr('[OrderManager] - ' + service_key + ': ' + response.message)
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    def sendBytesRequest(self, service_key): # TODO: TEMPORARY
        """
        General Trigger Request for any trigger-based service

        Returns TriggerResponse object
        """
        client = self._clients[service_key]

        request = SendBytesRequest(0)
        try:
            response = client(request)
            if response is not None: rospy.loginfo('[OrderManager] - ' + service_key + ': ' + response.data)
            else: rospy.logerr('[OrderManager] - ' + service_key + ': ' + response.data)
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    def sendVisionObjectDetectRequest(self):
        """
        Wrapper of sendTriggerRequest for Vision Object Detect with message parsing.

        Returns tuple (eg, ('red', 'triangle', 0.25, 0.05, 0.3)) if object exists; otherwise None
        """
        response = self.sendBytesRequest(service_key='vision_detect_object')

        if response and response.data:
            # Parse Message to get Information
            # message = response.data
            args = response.data.split()
            return (args[0], args[1], float(args[2]), float(args[3]), float(args[4]))
        else: return None


    def sendMoveToObjectRequest(self, x, y, z):
        """
        Send Custom Trigger for robot to move to object
        """
        service_key = 'motion_move_to_object'
        client = self._clients[service_key]

        request = MoveToObjectRequest(
            location=Point(
                x=x,
                y=y,
                z=z
            )
        )
        try:
            response = client(request)
            if response.success: rospy.loginfo('[OrderManager] - ' + service_key + ': ' + response.message)
            else: rospy.logerr('[OrderManager] - ' + service_key + ': ' + response.message)
            return response
        except rospy.ServiceException as exc:
            rospy.logerr('[OrderManager] Service did not process request: ' + str(exc))
            return None


    """
    ##########################
    CLASS SUBSCRIBER CALLBACKS
    ##########################
    """

    def handleProximityChangeCallback(self, msg, id):
        """
        If a container/object arrives to break beam, then we stop to process the new order
        """

        if not msg.data: return # Ignore objects leaving beams

        if id == 'in':
            # Stop Conveyor In
            response_conveyor = self.stopConveyorIn()
            if not response_conveyor.success:
                self.startConveyorIn()
                return

            # Get Vision Feedback
            rospy.loginfo('[OrderManager] Waiting for Vision Feedback...')
            response_object_detect = self.sendVisionObjectDetectRequest()
            if response_object_detect is None:
                self.startConveyorIn()
                return
            color, shape, x, y, z = response_object_detect
            rospy.loginfo('[OrderManager] Got Vision Feedback: color - ' + color + '\tshape - '+ shape + '\tx - ' + str(x) + '\ty - ' + str(y) + '\tz - ' + str(z))

            rospy.loginfo('[OrderManager] Checking if object is needed...')
            # Check Order is needed
            if not self._order_list.is_object_needed(color=color, shape=shape):
                rospy.loginfo('[OrderManager] Object is not needed...resuming conveyor')
                self.startConveyorIn()
                return
            rospy.loginfo('[OrderManager] Object is needed!')

            # Move to object
            rospy.loginfo('[OrderManager] Triggering Move To Object...')
            response_move_to_object = self.sendMoveToObjectRequest(x=x, y=y, z=z)
            if not (response_move_to_object and response_move_to_object.success):
                self.sendTriggerRequest(service_key='motion_move_to_container')
                self.startConveyorIn()
                return
            rospy.loginfo('[OrderManager] Successfully Moved to Object!')

            # Pickup Object
            rospy.loginfo('[OrderManager] Triggering Pickup Object')
            response_pickup_object = self.sendTriggerRequest(service_key='motion_pickup_object')
            if not (response_pickup_object and response_pickup_object.success):
                self.sendTriggerRequest(service_key='motion_move_to_container')
                self.startConveyorIn()
                return
            rospy.loginfo('[OrderManager] Successfully Picked up object!')

            # Move to container
            rospy.loginfo('[OrderManager] Triggering Move to Container...')
            response_move_to_container = self.sendTriggerRequest('motion_move_to_container')
            if not (response_move_to_container and response_move_to_container.success):
                self.sendTriggerRequest(service_key='motion_move_to_container')
                self.startConveyorIn()
                return
            rospy.loginfo('[OrderManager] Succesfully Moved to Container!')

            if not self._is_container_ready: self.waitForContainer()

            # Drop Object
            rospy.loginfo('[OrderManager] Triggering Drop Object to Container...')
            response_drop_object = self.sendTriggerRequest('motion_drop_object')
            if not (response_drop_object and response_drop_object.success):
                self.startConveyorIn()
                return
            rospy.loginfo('[OrderManager] Successfully Dropped Object to Container!')

            # Object Dropped Successfully
            # Update order
            is_order_done, new_order_ready = self._order_list.update_order()
            if is_order_done: self.startConveyorOut(spawn=new_order_ready)

            # Reset
            # self.sendTriggerRequest(service_key='motion_move_to_home')
            self.startConveyorIn()
            return



        elif id == 'out':
            response_conveyor = self.stopConveyorOut()
            return

        return


    """
    ################
    HELPER FUNCTIONS
    ################
    """

    def startConveyorIn(self):
        """Continue Spawning Objects && Moving Conveyor Belt"""
        self._publishers['spawner_set_auto'].publish(Bool(True))
        response = self.sendConveyorControlRequest(id='in', state='on')
        return response


    def stopConveyorIn(self):
        """Stop Spawning Objects && Stop Conveyor Belt"""
        self._publishers['spawner_set_auto'].publish(Bool(False))
        response = self.sendConveyorControlRequest(id='in', state='off')
        return response


    def startConveyorOut(self, spawn=False):
        """Activate Conveyor Belt Out; Spawn Container if necessary (if order exists)"""
        if spawn: self._publishers['spawner_create_container'].publish(Empty())
        response = self.sendConveyorControlRequest(id='out', state='on')
        self._is_container_ready = False
        return


    def stopConveyorOut(self):
        """Stop Conveyor Belt Out"""
        response = self.sendConveyorControlRequest(id='out', state='off')
        self._is_container_ready = True
        return response


    def waitForContainer(self):
        """Loops until container arrives"""
        rospy.loginfo('[OrderManager] Waiting for Container...')
        while not self._is_container_ready and not rospy.is_shutdown():
            self._rate.sleep()
        rospy.loginfo('[OrderManager] Container has arrived!')
        return

    """
    #################################
    MOCK SERVER FUNCTIONS FOR TESTING
    #################################
    """

    def handleMotionMoveToObjectRequest(self, request):
        """Mock Response Demo wait 5 seconds"""
        print(request)
        # point = request.location
        # rospy.logwarn(point.x, point.y, point.z)

        motionIsFinished = False
        counter = 0
        while not motionIsFinished and not rospy.is_shutdown():
            if counter >= 5.0 * SLEEP_RATE: motionIsFinished = True
            counter += 1
            # Publish Feedback??

            self._rate.sleep()

        response = MoveToObjectResponse(
            success=True,
            message="Robot move to object successfully"
        )

        return response

    def handleVisionDetectObjectRequest(self, request):
        """Mock Response"""

        response = SendBytesResponse(
            data="red triangle 0.0 0.5 0.25"
        )

        return response


    def handleMockTrigger(self, request):
        """Mock Response"""

        response = TriggerResponse(
            success=True,
            message="Successful Trigger Message"
        )

        return response



if __name__ == "__main__":
    rospy.init_node('order_manager')
    OrderManager()
    rospy.spin()
