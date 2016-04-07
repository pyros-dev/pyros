# -*- coding: utf-8 -*-
from __future__ import absolute_import

import contextlib
import time
from collections import namedtuple
import zmq
import pickle
#import dill as pickle
import inspect

try:
    from six import reraise
except ImportError:  # if six is not present we will not reraise the remote exception
    reraise = None

try:
    from tblib import Traceback
except ImportError:  # if tblib is not present, we will not be able to forward the traceback
    Traceback = None

from .message import ServiceRequest, ServiceResponse, ServiceResponse_dictparse, ServiceException, ServiceException_dictparse

from .master import manager
from .exceptions import UnknownResponseTypeException

# Lock is definitely needed ( not implemented in proxy objects, unless the object itself already has it, like Queue )
services_lock = manager.Lock()
services = manager.dict()


@contextlib.contextmanager
def service_provider_cm(node_name, svc_address, node_providers):
    # advertising services
    services_lock.acquire()
    for svc_name, svc_endpoint in node_providers.iteritems():
        #print('-> Providing {0} with {1}'.format(svc_name, svc_endpoint))
        # needs reassigning to propagate update to manager
        services[svc_name] = (services[svc_name] if svc_name in services else []) + [(node_name, svc_address)]
    services_lock.release()

    yield

    # concealing services
    services_lock.acquire()
    for svc_name, svc_endpoint in node_providers.iteritems():
        #print('-> Unproviding {0}'.format(svc_name))
        services[svc_name] = [(n, a) for (n, a) in services[svc_name] if n != node_name]
    services_lock.release()


class ServiceCallTimeout(Exception):
    pass


# TODO : make this pickleable so we can move it around ( and dynamically rebuild the socket connection )
class Service(object):

    @staticmethod
    # TODO : optionally adds more element from the signature ( num args, etc. )
    def discover(name, timeout=None, minimum_providers=1):
        """
        discovers a service. If timeout is specified, waits for at least minimum_providers service instance to be available.
        Note : we do not want to make the discovery block undefinitely since we never know for sure if a service is running or not
        TODO : improve with future...
        :param name: name of the service
        :param timeout: maximum number of seconds the discover can wait for a discovery matching requirements. if None, doesn't wait.
        :param minimum_providers the number of provider we need to reach before discover() returns if timeout enabled
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
        # TODO : make a provide just a list of node names, and have connection URLs somewhere else...

    # TODO : implement async_call ( and return future )
    def call(self, args=None, kwargs=None, node=None, send_timeout=1000, recv_timeout=5000, zmq_ctx=None):
        """
        Calls a service on a node with req as arguments. if node is None, a node is chosen by zmq.
        if zmq_ctx is passed, it will use the existing context
        Uses a REQ socket. Ref : http://api.zeromq.org/2-1:zmq-socket
        :param node : the node name

        """

        context = zmq_ctx or zmq.Context()
        assert isinstance(context, zmq.Context)

        args = args or ()
        assert isinstance(args, tuple)
        kwargs = kwargs or {}
        assert isinstance(kwargs, dict)

        socket = context.socket(zmq.REQ)

        # connect to all addresses ( optionally matching node name )
        for n, a in [(n, a) for (n, a) in self.providers if (not node or n == node)]:
            socket.connect(a)

        # build message
        fullreq = ServiceRequest(service=self.name, args=pickle.dumps(args), kwargs=pickle.dumps(kwargs))

        poller = zmq.Poller()
        poller.register(socket)  # POLLIN for recv, POLLOUT for send

        evts = dict(poller.poll(send_timeout))
        if socket in evts and evts[socket] == zmq.POLLOUT:
            socket.send(fullreq.serialize())
            # TODO : find a way to get rid fo these timeouts when debugging
            # TODO : when timeout Exception should occur ( not returning None )
            evts = dict(poller.poll(recv_timeout))  # blocking until answer
            if socket in evts and evts[socket] == zmq.POLLIN:
                resp = socket.recv()
                fullresp = ServiceResponse_dictparse(resp)

                if fullresp.has_field('response'):
                    return pickle.loads(fullresp.response)
                elif fullresp.has_field('exception'):
                    svcexc = fullresp.exception  # This has already been parsed by ServiceResponse_dictparse
                    tb = pickle.loads(svcexc.traceback)
                    if Traceback and isinstance(tb, Traceback):
                        reraise(pickle.loads(svcexc.exc_type), pickle.loads(svcexc.exc_value), tb.as_traceback())
                    else:  # traceback not usable
                        reraise(pickle.loads(svcexc.exc_type), pickle.loads(svcexc.exc_value), None)
                else:
                    raise UnknownResponseTypeException("Unknown Response Type {0}".format(type(fullresp)))
            else:
                raise ServiceCallTimeout("Did not receive response through ZMQ socket.")
        else:
            raise ServiceCallTimeout("Can not send request through ZMQ socket.")

        # TODO : exception if service disappeared in the mean time...

# convenience
discover = Service.discover
