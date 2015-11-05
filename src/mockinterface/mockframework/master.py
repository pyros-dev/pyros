# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

import multiprocessing




"""
Keeps a list of topics, services, params, nodes available on the system
Very very simple implementation for now, as it is not our main focus to build yet another full fledged distributed multiprocessing framework
"""
class Master:  # TODO : masterless if possible simply

    jobs = []
    params = {}

    def __init__(self):
        pass

    def launch(self, name, target, args):
        p = multiprocessing.Process(name=name, target=target, args=args)
        self.jobs.append(p)
        p.start()

    def terminate(self, name):
        # killing processes
        for j in [x for x in self.jobs if x.name == name]:
            j.terminate()
            j.join()
            print '%s.exitcode = %s' % (j.name, j.exitcode)
        # removing them from job list
        self.jobs[:] = [x for x in self.jobs if x.name == name]

    def set_param(self, name, value):
        self.params[name] = value

    def get_param(self, name):
        return self.params[name]



