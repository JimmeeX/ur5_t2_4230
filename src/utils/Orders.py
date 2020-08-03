#!/usr/bin/env python

import time

class Order():
    def __init__(self, category, value, goal, status='queued'):
        self._id = time.time()    # Unique Unix Timestamp
        self._category = category # color, shape
        self._value = value
        self._goal = goal
        self._qty = 0
        self._status = status # queued, doing, done

    @property
    def id(self):
        return self._id


    @property
    def category(self):
        return self._category


    @property
    def value(self):
        return self._value


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


    def add_order(self, id, category, value, qty, status):
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









