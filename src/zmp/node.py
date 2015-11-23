# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import
from __future__ import print_function

import sys
import tempfile
import multiprocessing, multiprocessing.reduction
import time
from collections import namedtuple
import zmq
import socket
import dill

from funcsigs import signature

# allowing pickling of exceptions to transfer it
from tblib.decorators import return_error, Error
# Serializer for nested classes and Exceptions ( and more ) :
# https://github.com/uqfoundation/dill
# TODO : evaluate replacing pickle + tblib + funcsigs + whatever specific serialization lib by
# TODO : - https://github.com/uqfoundation/dill
# TODO : - OR https://github.com/irmen/Serpent
# TODO : - OR https://github.com/esnme/ultrajson
# TODO : - OR directly into the protobuf format
# TODO : - OR something else ?

### IMPORTANT : COMPOSITION -> A SET OF NODE SHOULD ALSO 'BE' A NODE ###
### IMPORTANT : IDENTITY
### Category Theory https://en.wikipedia.org/wiki/Category_theory

### Data Flow with topic :
#
# object : message
# arrow : topic_listener
# binary op - associativity : listener l1, l2, l3 <= (l1 . l2) .l3 == l1. (l2. l3)
# binary op - identity : noop on listener => msg transfer as is
# -> Expressed in programming language (functional programming follows category theory) :
# msg3 = topic2_listener(topic1_listener(msg1))

# -> Expressed in graph :
# msg3 <--topic2_listener-- msg2 <--topic1_listener-- msg1

### RPC with services :
#
# object : (req, resp)
# arrow : service_call
# binary op - associativity : service s1, s2, s3 <= (s1 . s2) .s3 == s1. (s2. s3)
# binary op - identity : noop on service => (req, resp) transfer as is ( two ways comm )
# -> Expressed in programming language (functional programming follows category theory) :
#  (req3, resp3) = service2_call(service1_call((req1, resp1)))

# -> Expressed in graph :
# msg1 --service1_call--> msg2 --service2_call--> msg3

###### higher level for execution graph to be represented by a category ######

### Service is a first class citizen. node is abstracted in that perspective : implementation requires some discovery mechanism.
# object : service
# arrow : call
# binary op - associativity : call c1, c2, c3 <= (c1 . c2) .c3 == c1 . (c2 . c3)
# binary op - identity : ??? TODO
# -> Expressed in programming language (functional programming follows category theory) :
#  svc1 = lambda x: return ( lambda y: return svc3(y) )( x )  <= currying/partial => svc1 = lambda x, y : return svc23(x,y)

# -> Expressed in graph :
# svc1 --call--> svc2 --call--> svc3

### Node is a first class citizen
# object : node
# arrow : topic_listener / callback
# binary op - associativity : callback cb1. cb2. cb3 <= (cb1 . cb2) . cb3 == cb1 . (cb2 . cb3)
# binary op - identity : ??? TODO
# -> Expressed in programming language (functional programming follows category theory) :
#  node1.cb = lambda x: return ( lambda y: return node3.cb(y) )(x) <= currying/partial => node1.cb = lambda x, y : return node23(x,y)

# -> Expressed in graph :
# node3 <--topic_cb-- node2 <--topic_cb-- node1

from .exceptions import UnknownServiceException, UnknownRequestTypeException
from .message import ServiceRequest, ServiceRequest_dictparse, ServiceResponse
from .service import services, services_lock
#from .service import RequestMsg, ResponseMsg, ErrorMsg  # only to access message types

current_node = multiprocessing.current_process


# TODO : Nodelet ( thread, with fast intraprocess zmq comm - entity system design /vs/threadpool ?)

class Node(multiprocessing.Process):

    def __init__(self, name='node', socket_bind=None):
        """
        Initializes a Process
        :param name: Name of the node
        :param zmqbind: the string describing how to bind the ZMQ socket ( IPC, TCP, etc. )
        :return:
        """
        # TODO check name unicity
        super(Node, self).__init__(name=name)
        self.exit = multiprocessing.Event()
        self.listeners = {}
        self._providers_endpoint = []  # TODO : automatic detection
        self.tmpdir = tempfile.mkdtemp(prefix='zmp-' + self.name + '-')
        # if no socket is specified the services of this node will be available only through IPC
        self._svc_address = socket_bind if socket_bind else 'ipc://' + self.tmpdir + '/services.pipe'

    def provides(self, svc_callback):
        # TODO : multiple endpoint for one service ( can help in some specific cases )
        self._providers_endpoint.append(svc_callback)

    def start(self):
        """
        Start child process
        """
        if self._popen is not None:
            # if already started, we shutdown and join before restarting
            self.shutdown(join=True)
            self.start()
        else:
            super(Node, self).start()

    def run(self):
        print('Starting {node} [{pid}] => {address}'.format(node=self.name, pid=self.pid, address=self._svc_address))

        zcontext = zmq.Context()  # check creating context in init ( compatibility with multiple processes )
        zcontext.setsockopt(socket.SO_REUSEADDR, 1)  # required to make restart easy and avoid debugging traps...
        svc_socket = zcontext.socket(zmq.REP)  # Ref : http://api.zeromq.org/2-1:zmq-socket # TODO : ROUTER instead ?
        svc_socket.bind(self._svc_address,)

        poller = zmq.Poller()
        poller.register(svc_socket, zmq.POLLIN)

        global services, services_lock
        # advertising services
        services_lock.acquire()
        for svc_callback in self._providers_endpoint:
            print('-> Providing {0} with {1}'.format(svc_callback.func_name, svc_callback))
            # needs reassigning to propagate update to manager
            services[svc_callback.func_name] = (services[svc_callback.func_name] if svc_callback.func_name in services else []) + [(self.name, self._svc_address, dill.dumps(svc_callback))]
        services_lock.release()

        # loop listening to connection
        while not self.exit.is_set():
            socks = dict(poller.poll(timeout=100))  # blocking. messages are received ASAP. timeout only determine shutdown speed.
            if svc_socket in socks and socks[svc_socket] == zmq.POLLIN:
                try:
                    print('-> POLLIN on {0}'.format(svc_socket))
                    req = ServiceRequest_dictparse(svc_socket.recv())
                    if isinstance(req, ServiceRequest):  # TODO : check function signature ( not only name )
                        providers = {srv.func_name: srv for srv in self._providers_endpoint}
                        if req.service and req.service in providers.keys():

                            args = dill.loads(req.args) if req.args else ()
                            # add 'self' if this is a bound method.
                            #if self.__self__ is not None:
                            #    args = (self.__self__, ) + tuple(args)
                                #TODO : check this on node methods...
                            kwargs = dill.loads(req.kwargs) if req.kwargs else {}

                            # This will grab all exceptions in there and encapsulate as Error type
                            try:
                                resp = (providers[req.service])(*args, **kwargs)
                            except Exception:
                                resp = Error(*sys.exc_info())
                            svc_socket.send(ServiceResponse(
                                type=ServiceResponse.ERROR if isinstance(resp, Error) else ServiceResponse.RESPONSE,
                                service=req.service,
                                response=dill.dumps(resp)
                            ).serialize())
                        else:
                            raise UnknownServiceException("Unknown Service {0}".format(req.service))
                    else:
                        raise UnknownRequestTypeException("Unknown Request Type {0}".format(type(req.request)))
                except (UnknownServiceException, UnknownRequestTypeException):
                    # we just transmit node known errors, and keep spinning...
                    known_error = Error(*sys.exc_info())
                    svc_socket.send(ServiceResponse(
                                type=ServiceResponse.ERROR,
                                service=req.service,
                                response=dill.dumps(known_error)
                    ).serialize())

        # deadvertising services
        services_lock.acquire()
        for svc_callback in self._providers_endpoint:
            print('-> Unproviding {0}'.format(svc_callback.func_name))
            services[svc_callback.func_name] = [(n, a, s) for (n, a, s) in services[svc_callback.func_name] if n != self.name]
        services_lock.release()

        print("You exited!")

    def shutdown(self, join=True):
        """
        Clean shutdown of the node.
        :param join: optionally wait for the process to end (default : True)
        :return: None
        """
        if self._popen is not None:  # check if process started
            print("Shutdown initiated")
            self.exit.set()
            if join:
                self.join()
                # TODO : after terminate, not before
                self._popen = None  # this should permit start to run again
            # TODO : timeout before forcing terminate (SIGTERM)
        pass

    def listen(self, topic, callback):

        # TODO actually open comm channels

        # find topic sender

        # build a pipe with sender

        # register the listener
        self.listeners[topic] = callback
        pass

    def unlisten(self, topic):
        #TODO : inverse of listen
        pass


