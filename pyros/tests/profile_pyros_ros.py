#!/usr/bin/env python
from __future__ import absolute_import

# ROS SETUP if needed
import logging
import multiprocessing
import time
import cProfile

import sys
from pyros import PyrosROS
import rospy
import rosgraph
import roslaunch

roscore_process = None

# BROKEN ?? start roscore beofre running this...
# if not rosgraph.masterapi.is_online():
#     # Trying to solve this : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
#     def ros_core_launch():
#         roslaunch.main(['roscore', '--core'])  # same as rostest_main implementation
#
#     roscore_process = multiprocessing.Process(target=ros_core_launch)
#     roscore_process.start()
#
#     while not roscore_process.is_alive():
#         time.sleep(0.2)  # waiting for roscore to be born
#
#     assert roscore_process.is_alive()

assert rosgraph.masterapi.is_online()

# Start roslaunch
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

# starting connection cache is available
rospy.set_param('/connection_cache/spin_freq', 2)  # 2 Hz
connection_cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py', name='connection_cache',
                                                 remap_args=[('~list', '/pyros_ros/connections_list'),
                                                             ('~diff', '/pyros_ros/connections_diff'),
                                                             ])
try:
    connection_cache_proc = launch.launch(connection_cache_node)
except roslaunch.RLException as rlexc:
    pass  # ignore

time.sleep(2)

# start a bunch of node (this will load ros interface)
pub_proc = []

def start_pub_node(pubnum):
    node_name = 'string_pub_node_' + str(pubnum)
    rospy.set_param('/' + node_name + '/topic_name', 'pub_' + str(pubnum))
    rospy.set_param('/' + node_name + '/test_message', 'msg_' + str(pubnum))
    node = roslaunch.core.Node('pyros_test', 'string_pub_node.py', name=node_name)
    try:
        pub_proc.append(launch.launch(node))
    except roslaunch.RLException as rlexc:
        logging.error(
            "pyros_test is needed to run this. Please verify that it is installed in your ROS environment")
        raise

# TODO : make MANY nodes / services / params to simulate complex robot and make profiling more realistic.
time.sleep(2)  # waiting for nodes to be up

rosn = PyrosROS()

rosn.setup(
    services=['/test/empsrv', '/test/trgsrv'],
    params=['/test/confirm_param'],
    enable_cache=connection_cache_proc.is_alive()
)

print("Module LOADED")


def update_loop():
    total_count = 1024*1024*255
    count = 0
    start = time.time()
    pct = 0
    last_pct = -1
    max_pubnodes = 42
    node_step = 0
    last_node_step = -1
    while count < total_count:
        # time is ticking
        now = time.time()
        timedelta = now - start
        start = now

        rosn.update(timedelta)

        count += 1

        # creating and removing nodes while looping
        node_step = count * max_pubnodes * 2/ total_count
        if node_step != last_node_step:
            last_node_step = node_step
            if count < total_count/2:
                # adding nodes
                print("adding node {0}".format(node_step))
                start_pub_node(node_step)

            elif pub_proc:
                # stopping nodes LIFO
                print("stopping node {0}".format(len(pub_proc)-1))
                pub_proc.pop().stop()

        pct = count * 100 / total_count
        if pct != last_pct:
            last_pct = pct
            sys.stdout.write("\r" + str(last_pct) + "%")
            sys.stdout.flush()

# In case you want to run kernprof here
#update_loop()
cProfile.run('update_loop()', sort='cumulative')

# ensuring all process are finished
for p in pub_proc:
    p.stop()

if connection_cache_proc is not None:
    connection_cache_proc.stop()

rospy.signal_shutdown('test complete')

if roscore_process is not None:
    roscore_process.terminate()  # make sure everything is stopped