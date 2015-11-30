# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import
from __future__ import print_function

import sys
import tempfile
import multiprocessing, multiprocessing.reduction
import types
import zmq
import socket
import pickle
#import dill as pickle

# allowing pickling of exceptions to transfer it
from collections import namedtuple
try:
    from tblib.decorators import Traceback
    # TODO : potential candidates for pickle + tblib replacement for easier serialization
    # TODO : - https://github.com/uqfoundation/dill
    # TODO : - OR https://github.com/cloudpipe/cloudpickle
    # TODO : - OR https://github.com/irmen/Serpent ?
    # TODO : - OR https://github.com/esnme/ultrajson ?
    # TODO : - OR something else ?
except ImportError:
    Traceback = None

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

from .master import manager
from .exceptions import UnknownServiceException, UnknownRequestTypeException
from .message import ServiceRequest, ServiceRequest_dictparse, ServiceResponse, ServiceException
from .service import services, services_lock
#from .service import RequestMsg, ResponseMsg, ErrorMsg  # only to access message types

current_node = multiprocessing.current_process

# Lock is definitely needed ( not implemented in proxy objects, unless the object itself already has it, like Queue )
nodes_lock = manager.Lock()
nodes = manager.dict()


# TODO : Nodelet ( thread, with fast intraprocess zmq comm - entity system design /vs/threadpool ?)

class Node(multiprocessing.Process):

    EndPoint = namedtuple("EndPoint", "self func")

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
        self._providers = {}
        self.tmpdir = tempfile.mkdtemp(prefix='zmp-' + self.name + '-')
        # if no socket is specified the services of this node will be available only through IPC
        self._svc_address = socket_bind if socket_bind else 'ipc://' + self.tmpdir + '/services.pipe'

    def provides(self, svc_callback, service_name=None):
        service_name = service_name or svc_callback.__name__
        # we store an endpoint ( bound method or unbound function )
        self._providers[service_name] = Node.EndPoint(
                self=getattr(svc_callback, '__self__', None),
                func=getattr(svc_callback, '__func__', svc_callback),
        )

    def withholds(self, service_name):
        service_name = getattr(service_name, '__name__', service_name)
        # we store an endpoint ( bound method or unbound function )
        self._providers.pop(service_name)

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
        for svc_name, svc_endpoint in self._providers.iteritems():
            print('-> Providing {0} with {1}'.format(svc_name, svc_endpoint))
            # needs reassigning to propagate update to manager
            services[svc_name] = (services[svc_name] if svc_name in services else []) + [(self.name, self._svc_address)]
        services_lock.release()

        # advertise itself
        nodes_lock.acquire()
        nodes[self.name] = {'service_conn': self._svc_address}
        nodes_lock.release()

        # loop listening to connection
        while not self.exit.is_set():
            # blocking. messages are received ASAP. timeout only determine shutdown speed.
            socks = dict(poller.poll(timeout=100))
            if svc_socket in socks and socks[svc_socket] == zmq.POLLIN:
                req = None
                try:
                    print('-> POLLIN on {0}'.format(svc_socket))
                    req = ServiceRequest_dictparse(svc_socket.recv())
                    if isinstance(req, ServiceRequest):
                        if req.service and req.service in self._providers.keys():

                            args = pickle.loads(req.args) if req.args else ()
                            # add 'self' if providers[req.service] is a bound method.
                            if self._providers[req.service].self:
                                args = (self, ) + args
                            kwargs = pickle.loads(req.kwargs) if req.kwargs else {}

                            resp = self._providers[req.service].func(*args, **kwargs)
                            svc_socket.send(ServiceResponse(
                                service=req.service,
                                response=pickle.dumps(resp)
                            ).serialize())

                        else:
                            raise UnknownServiceException("Unknown Service {0}".format(req.service))
                    else:
                        raise UnknownRequestTypeException("Unknown Request Type {0}".format(type(req.request)))
                except Exception:  # we transmit back all errors, and keep spinning...
                    exctype, excvalue, tb = sys.exc_info()
                    if Traceback is not None:
                        svc_socket.send(ServiceException(
                            service=req.service if req else "Unknown",
                            exc_type=pickle.dumps(exctype),
                            exc_value=pickle.dumps(excvalue),
                            traceback=pickle.dumps(Traceback(tb)),
                        ).serialize())
                    else:
                        svc_socket.send(ServiceException(
                            service=req.service if req else "Unknown",
                            exc_type=pickle.dumps(exctype),
                            exc_value=pickle.dumps(excvalue),
                            traceback=pickle.dumps("Traceback Unavailable. python-tblib needs to be installed."),
                        ).serialize())

        # concealing services
        services_lock.acquire()
        for svc_name, svc_endpoint in self._providers.iteritems():
            print('-> Unproviding {0}'.format(svc_name))
            services[svc_name] = [(n, a) for (n, a) in services[svc_name] if n != self.name]
        services_lock.release()

        # concealing itself
        nodes_lock.acquire()
        nodes[self.name] = {}
        nodes_lock.release()

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


