#!/usr/bin/env python


import cProfile
from zmp import Node


nprof = Node()

cProfile.run('nprof.run()')

