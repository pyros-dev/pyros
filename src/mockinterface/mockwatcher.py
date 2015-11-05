# -*- coding: utf-8 -*-
from __future__ import absolute_import, print_function
from concurrent import futures
import threading
import socket



class ROSWatcher(threading.Thread):  #TODO : DO NOT inherit from thread. instead use the executor for watching.

    def __init__(self, topics_change_cb, services_change_cb, actions_change_cb):  # TODO : use Queue for callbacks to be executed in main thread.
        super(ROSWatcher, self).__init__()

        self.executor = futures.ThreadPoolExecutor(max_workers=1)

        #TODO : acessing members should be done via property that copy the object to avoid accidental modification during use

        self._stop_lambda = stop_lambda

        self.publishers = []
        self.subscribers = []
        self.services = []
        self.topics_change_cb = topics_change_cb
        self.services_change_cb = services_change_cb
        self.actions_change_cb = actions_change_cb

    def run(self):
        """
        Starting this thread asynchronously
        """
        rate = rospy.Rate(1)  # 1hz
        # Night gathers, and now my watch begins. It shall not end until my death.
        # I shall take no wife, hold no lands, father no children.
        # I shall wear no crowns and win no glory.
        # I shall live and die at my post.
        # I am the sword in the darkness.
        # I am the watcher on the walls.
        # I am the fire that burns against cold, the light that brings the dawn, the horn that wakes the sleepers, the shield that guards the realms of men.
        # I pledge my life and honor to the Night's Watch, for this night and all the nights to come
        while not stop_lambda():
            self.update()
            rate.sleep()  # sleeping to not panic the master

    def update(self):
        """
        Update function to call from a looping thread.
        """

        try:
            publishers, subscribers, services = self._master.getSystemState()

            new_publishers = [pub[0] for pub in publishers if pub not in self.publishers]
            lost_publishers = [pub[0] for pub in self.publishers if pub not in publishers]

            new_services = [srv[0] for srv in services if srv not in self.services]
            lost_services = [srv[0] for srv in self.services if srv not in services]

            if len(new_publishers) > 0 or len(lost_publishers) > 0:
                self.topics_change_cb(new_publishers, lost_publishers)

            if len(new_services) > 0 or len(lost_services) > 0:
                self.services_change_cb(new_services, lost_services)

            # TODO : find a simple way to detect actions ( or drop support for it if it s too much useless code ? )

            self.publishers = publishers
            self.subscribers = subscribers
            self.services = services

        except socket.error:
            rospy.logerr("Gateway : couldn't get system state from the master "
                         "[did you set your master uri to a wireless IP that just went down?]")
