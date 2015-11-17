# -*- coding: utf-8 -*-
from __future__ import absolute_import

import time
from collections import namedtuple
import zmq

from .master import manager
from .exceptions import UnknownResponseTypeException

# Lock is definitely needed ( not implemented in proxy objects, unless the object itself already has it, like Queue )
services_lock = manager.Lock()
services = manager.dict()

# need to be at the module level to be pickleable
RequestMsg = namedtuple("RequestMsg", "service request")
ResponseMsg = namedtuple("ResponseMsg", "service response")
ErrorMsg = namedtuple("ErrorMsg", "service error")


class Service(object):

    @staticmethod
    def discover(name, timeout=None, minimum_providers=1):
        """
        discovers a service. wait for at least one service instance to be available.
        :param name: name of the service
        :param minimum_providers the number of provider we need to reach before discover() returns
        :param timeout: maximum number of seconds the discover can wait for a discovery matching requirements. if None, doesn't wait.
        :return: a Service object, containing the list of providers. if the minimum number cannot be reached, still returns what is available.
        """
        start = time.time()
        endtime = timeout if timeout else 0

        while True:
            timed_out = time.time() - start > endtime
            if name in services and isinstance(services[name], list):
                if len(services[name]) >= minimum_providers or timed_out:
                    providers = services[name]
                    if providers:
                        return Service(name, providers)
                    else:
                        return None

            if timed_out:
                break
            # else we keep looping after a short sleep ( to allow time to refresh services list )
            time.sleep(0.2)  # sleep
        return None

    def __init__(self, name, providers=None):
        self.name = name
        self.providers = providers

    def call(self, req, node=None, send_timeout=1000, recv_timeout=5000, zmq_ctx=None):
        """
        Calls a service on a node with req as arguments. if node is None, a node is chosen by zmq.
        if zmq_ctx is passed, it will use the existing context
        :param node : the node name
        """

        context = zmq_ctx or zmq.Context()
        assert isinstance(context, zmq.Context)

        print "Connecting to server..."
        socket = context.socket(zmq.REQ)

        # connect to all addresses ( optionally matching node name )
        for a in [a for (n, a) in self.providers if (not node or n == node)]:
            socket.connect(a)

        # build message
        fullreq = RequestMsg(service=self.name, request=req)

        poller = zmq.Poller()
        poller.register(socket)  # POLLIN for recv, POLLOUT for send

        evts = dict(poller.poll(send_timeout))
        if socket in evts and evts[socket] == zmq.POLLOUT:
            print "POLLOUT"
            socket.send_pyobj(fullreq)
            evts = dict(poller.poll(recv_timeout))  # blocking until answer
            if socket in evts and evts[socket] == zmq.POLLIN:
                print "POLLIN"
                fullresp = socket.recv_pyobj()

                if isinstance(fullresp, ErrorMsg):
                    fullresp.error.reraise()
                elif isinstance(fullresp, ResponseMsg):
                    return fullresp.response
                else:
                    raise UnknownResponseTypeException("Unknown Response Type {0}".format(type(req.response)))

        # TODO : exception on timeout
        return None

    def expose(self):
        pass

    def hide(self):
        pass

# convenience
discover = Service.discover
