# -*- coding: utf-8 -*-
from __future__ import absolute_import

from collections import namedtuple


"""
Protocol allowing dynamic specification of message format
"""
# CAREFUL : topic might not be a complete self sufficient concept ( like service )
# TODO : study PIPELINE ( and others ) from zmq
# GOAL : find concept that allow like service ( remote version of funciton call ) but with inverted control flow ( callback )

def gen_msg_type(self, name, **kwargs):
    return namedtuple(name, **kwargs)




class Topic(object):

    def __init__(self, name):
        self.name = name
        self.cur_msg = None

    def publish(self, msg):
        self.cur_msg = msg
        return True



