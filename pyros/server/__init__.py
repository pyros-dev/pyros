# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function

"""
This is hte server side of pyros
client-Server Architecture was chosen because it enforces direction of knowledge : 
 - the client knows the server
 - the server doesn't know the client(s)
The design is to have only one server/node per multiprocess system we want to interface with.
client will be able to send requests to them.

TODO : requests will setup stream (Functional Reactive Programming / HTTP2 style)
to allow pushing data from server to connected client
"""