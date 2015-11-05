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

class Service(object):

    def __init__(self, name, callback):
        self.name = name
        self.callback = callback

    def call(self, req, node=None):

        # find the nodes that have this service
        prov = [(n, p) for (s, n, p) in services if s == self]

        if node:
            dests = [(n, p) for (n, p) in prov if n == node]
            node, pipe = dests[0]
        else:
            node, pipe = prov[0]  # just pick one (discovery ? random ?)

        # build message
        fullreq = Request(origin=multiprocessing.current_process().name, destination=node, payload=req)

        pipe.send(fullreq)
        fullresp = pipe.recv()

        return fullresp.payload


