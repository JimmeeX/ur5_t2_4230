#!/usr/bin/env python

import time

class Order():
    def __init__(self, color, shape, goal, status='queued'):
        self._id = int(time.time() * 1000)  # Unique Unix Timestamp in milliseconds
        self._color = color                 # Any valid color or 'none'
        self._shape = shape                 # Any valid shape or 'none'
        self._goal = goal                   # Target Quantity
        self._qty = 0                       # Current Quantity
        self._status = status               # queued, doing, done

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


    @property
    def status(self):
        return self._status


    def inc_qty(self):
        """Increment Quantity when a new object has been placed"""
        self._qty += 1


    def set_status(self, status):
        self._status = status


class OrderList():
    def __init__(self, valid_colors, valid_shapes):
        self._orders = []
        self._order_doing = None
        self._valid_colors = valid_colors
        self._valid_shapes = valid_shapes


    @property
    def orders_queued(self):
        return list(filter(lambda o: o.status == 'queued', self._orders))


    @property
    def orders_doing(self):
        if self._order_doing is None: return []
        return [self._order_doing]


    @property
    def orders_done(self):
        return list(filter(lambda o: o.status == 'done', self._orders))


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
        if self._order_doing is None:
            order = Order(
                color=color,
                shape=shape,
                goal=goal,
                status='doing'
            )
            triggerSpawnContainer = True
            self._order_doing = order
        else:
            order = Order(
                color=color,
                shape=shape,
                goal=goal,
                status='queued'
            )
            triggerSpawnContainer = False

        self._orders.append(order)

        return (True, 'Order added successfully', triggerSpawnContainer)


    def delete_order(self, id):
        """
        - Can only remove orders that are 'queued'
        """
        new_orders = list(filter(lambda o: o.id == id and o.status == 'queued', self._orders))
        if len(new_orders) == len(self._orders) - 1:
            self._orders = new_orders.copy()
            return (True, 'Order deleted successfully')
        else:
            return (False, 'Failed to delete order')


    def is_object_needed(self, color, shape):
        """Checks if object is needed for current order"""
        if self._order_doing is None: return False

        if self._order_doing.color != 'none':
            if color != self._order_doing.color: return False

        if self._order_doing.shape == 'none':
            if shape != self._order_doing.shape: return False

        return True

    def finish_order(self):
        """
        Sets current order to done, and takes in the next order in the queue to work on
        """
        # TODO
        pass


    def _get_orders_by_status(self, status):
        # TODO
        return list(sorted(filter(lambda o: o.status == status, self._orders), key=lambda o: o.id))


    # def get_orders_by_status(self):









