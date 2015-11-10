# -*- coding: utf-8 -*-
from __future__ import absolute_import

from collections import namedtuple
import multiprocessing


"""
Protocol allowing dynamic specification of message format
"""

from . import services, services_lock

def gen_msg_type(self, name, **kwargs):
    return namedtuple(name, **kwargs)

Request = namedtuple("Request", "origin destination payload")
Response = namedtuple("Response", "origin destination payload")


def discover(name):
    return Service(name, [(n, p) for (s, n, p) in services if s == name])


class Service(object):

    def __init__(self, name, providers):
        self.name = name
        self.providers = providers

    def call(self, req, node=None):
        """
        Calls a service on a node with req as arguments. if node is None, a node is chosen.
        TODO : implement a strategy to choose the node ( round robin, load balance, etc. )
        """

        if node:
            dests = [(n, p) for (n, p) in self.providers if n == node]
            node, pipe = dests[0]
        else:
            node, pipe = self.providers[0]  # just pick one (discovery ? random ?)

        # build message
        fullreq = Request(origin=multiprocessing.current_process().name, destination=node, payload=req)

        pipe.send(fullreq)
        fullresp = pipe.recv()

        return fullresp.payload

    def expose(self):
        pass

    def hide(self):
        pass
