#!/usr/bin/env python

import time

class Order():
    def __init__(self, color, shape, goal):
        self._id = int(time.time() * 1000)  # Unique Unix Timestamp in milliseconds
        self._color = color                 # Any valid color or 'none'
        self._shape = shape                 # Any valid shape or 'none'
        self._goal = goal                   # Target Quantity
        self._qty = 0                       # Current Quantity

    @property
    def id(self):
        return self._id


    @property
    def color(self):
        return self._color


    @property
    def shape(self):
        return self._shape


    @property
    def goal(self):
        return self._goal


    @property
    def qty(self):
        return self._qty


    def inc_qty(self):
        """Increment Quantity when a new object has been placed"""
        self._qty += 1


class OrderList():
    def __init__(self, valid_colors, valid_shapes):
        self._orders_queued = []
        self._orders_doing = [] # List, but for this implementation, there will only be one order 'doing' at a time
        self._orders_done = []
        self._valid_colors = valid_colors
        self._valid_shapes = valid_shapes


    @property
    def orders_queued(self):
        """Gets queue of orders (sorted oldest first)"""
        return self._orders_queued


    @property
    def orders_doing(self):
        return self._orders_doing


    @property
    def orders_done(self):
        """Gets list of finished orders (sorted newest first)"""
        return self._orders_done


    def add_order(self, color, shape, goal):
        """
        Returns
        (success, message, boolean to spawn container)
        """
        # Validation Check
        if not color in self._valid_colors: return (False, 'Not a valid color', False)
        if not shape in self._valid_shapes: return (False, 'Not a valid shape', False)
        if not goal > 0: return (False, 'Please enter a goal greater than 0', False)

        # Valid Order

        # Check if there is an order that is currently 'doing'
        if len(self._orders_doing) == 0:
            # Add it straight to the 'doing' queue
            order = Order(
                color=color,
                shape=shape,
                goal=goal,
            )
            triggerSpawnContainer = True
            self._orders_doing.append(order)
        else:
            # Add it to the waiting queue
            order = Order(
                color=color,
                shape=shape,
                goal=goal,
            )
            triggerSpawnContainer = False
            self._orders_queued.append(order)

        return (True, 'Order added successfully', triggerSpawnContainer)


    def delete_order(self, id):
        """
        - Can only remove orders that are 'queued'
        """
        new_orders = list(filter(lambda o: o.id == id, self._orders_queued))
        if len(new_orders) == len(self._orders_queued) - 1:
            self._orders_queued = new_orders.copy()
            return (True, 'Order deleted successfully')
        else:
            return (False, 'Failed to delete order')


    def is_object_needed(self, color, shape):
        """Checks if object is needed for current order"""
        if len(self._orders_doing) == 0: return False

        order = self._orders_doing[0]
        if order.color != 'none':
            if color != order.color: return False

        if order.shape != 'none':
            if shape != order.shape: return False

        return True

    def update_order(self):
        """
        Updates current order, checks if it's finished, and takes in the next order in the queue to work on if needed

        Returns Tuple
        - is_order_done
        - new_order_ready
        """
        is_order_done = False
        new_order_ready = False

        if len(self._orders_doing) == 0: return (False, False)

        # Update Current Order
        order = self._orders_doing[0]
        order.inc_qty()

        if order.qty == order.goal:
            is_order_done = True

            # Move order from doing to done
            order = self._orders_doing.pop()
            self._orders_done.insert(0, order) # Insert at beginning to enforce newest done at beginning of list

            # Take the next item in the queue to doing if exists
            if len(self._orders_queued) > 0:
                new_order_ready = True
                new_order = self._orders_queued.pop(0)
                self._orders_doing.append(new_order)

        return is_order_done, new_order_ready
