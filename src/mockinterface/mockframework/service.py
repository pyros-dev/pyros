# -*- coding: utf-8 -*-
from __future__ import absolute_import

import time
from collections import namedtuple
import multiprocessing, multiprocessing.reduction
import zmq
import pickle  # TODO : json, protobuf

"""
Protocol allowing dynamic specification of message format
"""

from . import services

def gen_msg_type(self, name, **kwargs):
    return namedtuple(name, **kwargs)

Request = namedtuple("Request", "service request")
Response = namedtuple("Response", "service response")


def discover(name, timeout=None):
    """
    discovers a service. optionally wait for at least one service instance to be available.
    :param name:
    :param timeout:
    :return:
    """
    start = time.clock()  # using clock instead of time to not be affected by other process running simultaneously
    endtime = timeout if timeout else 0

    while True:
        if name in services and services[name]:
            svc = services[name]
            return Service(name, svc)
        elif time.clock() - start > endtime:  # check for timeout
            break
        # else we keep looping
    return None


class Service(object):

    def __init__(self, name, providers=None):
        self.name = name
        self.providers = providers

    def call(self, req, node=None, send_timeout = 5, recv_timeout=5, zmq_ctx=None):
        """
        Calls a service on a node with req as arguments. if node is None, a node is chosen by zmq.
        if zmq_ctx is passed, it will use the existing context
        """

        context = zmq_ctx or zmq.Context()
        assert isinstance(context, zmq.Context)

        print "Connecting to server..."
        socket = context.socket(zmq.REQ)

        # connect to all addresses ( optionally matching node name )
        for a in [a for (n, a) in self.providers if (not node or n == node)]:
            socket.connect(a)

        # build message
        fullreq = Request(service=self.name, request=req)

        poller = zmq.Poller()
        poller.register(socket)  # POLLIN for recv, POLLOUT for send

        evts = poller.poll(send_timeout)
        if socket in evts and evts[socket] == zmq.POLLOUT:
            socket.send(pickle.dumps(fullreq))
            evts = poller.poll(recv_timeout)
            if socket in evts and evts[socket] == zmq.POLLIN:
                fullresp = pickle.loads(socket.recv())
                return fullresp.response

        #TODO : exception on timeout
        return None

    def expose(self):
        pass

    def hide(self):
        pass
