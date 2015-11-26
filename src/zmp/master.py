# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

import multiprocessing

#https://pymotw.com/2/multiprocessing/communication.html
#https://docs.python.org/2/library/multiprocessing.html#proxy-objects



"""
Keeps a list of topics, services, params, nodes available on the system
Very very simple implementation for now, as it is not our main focus to build yet another full fledged distributed multiprocessing framework
"""
manager = multiprocessing.Manager()


