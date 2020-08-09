#!/usr/bin/env python

import time

class Order():
    def __init__(self, color, shape, goal, status='queued'):
        self._id = time.time()  # Unique Unix Timestamp
        self._color = color     # Any valid color or 'none'
        self._shape = shape     # Any valid shape or 'none'
        self._goal = goal       # Target Quantity
        self._qty = 0           # Current Quantity
        self._status = status   # queued, doing, done

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
    def __init__(self, _orders_=[]):
        self._orders = []


    def add_order(self, id, color, shape, qty, status):
        # TODO
        pass


    def get_order(self, id):
        # TODO
        pass


    def delete_order(self, id):
        # TODO
        pass


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









