# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

import multiprocessing
import time
from collections import namedtuple


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


from . import services, services_lock
from .service import Request, Response

current_node = multiprocessing.current_process


class Node(multiprocessing.Process):
    EndPoint = namedtuple("EndPoint", "node_conn, service_callback")

    def __init__(self, name='node'):
        # TODO check name unicity
        super(Node, self).__init__(name=name)
        self.exit = multiprocessing.Event()
        self.listeners = {}
        self._providers_endpoint = {}

    def provides(self, service_name, service_callback):
        # actually open comm channels
        node_conn, client_conn = multiprocessing.Pipe()

        # TODO : multiple endpoint for one service ( can help in some speific cases )
        self._providers_endpoint[service_name] = self.EndPoint(node_conn, service_callback)

        # register the service for discovery
        services_lock.acquire()
        services.append((service_name, self.name, client_conn))
        services_lock.release()

    def run(self):
        print 'Starting %s' % self.name
        # loop listening to connection
        while not self.exit.is_set():
            for svc, ep in self._providers_endpoint.iteritems():
                try:
                    # TODO : check event based / proactor/ reactor design to optimize and avoid polling...
                    if ep.node_conn.poll(0.5):
                        req = ep.node_conn.recv()
                        resp = ep.callback(req.payload)
                        ep.node_conn.send(Response(origin=multiprocessing.current_process(), destination=req.origin, payload=resp))
                    else:  # no data, no worries.
                        pass
                except EOFError:  # empty pipe, no worries.
                    pass
                except Exception, e:
                    raise e
            time.sleep(0.1)  # avoid spinning too fast

        print "You exited!"

    def shutdown(self, join=True):
        print "Shutdown initiated"
        self.exit.set()
        if join:
            self.join()

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


