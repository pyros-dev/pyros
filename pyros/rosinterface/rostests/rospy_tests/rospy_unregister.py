#!/usr/bin/env python
from __future__ import absolute_import

import multiprocessing
import nose
import unittest
from nose.tools import assert_equal, assert_true, nottest
import time

try:
    import rospy
    import rosgraph
    import roslaunch
    import std_msgs.msg as std_msgs
except ImportError as exc:  # this will except only if you didnt setup ROS environment first
    import os
    import pyros_setup
    import sys
    # pass here the path to your workspace as argument
    sys.modules["pyros_setup"] = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', '..'))
    import rospy
    import rosgraph
    import roslaunch
    import std_msgs.msg as std_msgs

launch = None
roscore_process = None

# ############################################################################ #
# ### The aim of this file is to test strange behavior on core ROS modules ### #
# ############################################################################ #


# setup / teardown for EACH test to avoid side effects in same process
@nottest
def setup():
    if not rosgraph.masterapi.is_online():
        global roscore_process

        # Trying to solve this : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
        def ros_core_launch():
            roslaunch.main(['roscore', '--core'])  # same as rostest_main implementation

        roscore_process = multiprocessing.Process(target=ros_core_launch)
        roscore_process.start()

    # hack needed to wait for master until fix for https://github.com/ros/ros_comm/pull/711 is released
    roslaunch.rlutil.get_or_generate_uuid(None, True)

    # Start roslaunch
    global launch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # we still need a node to interact with topics
    rospy.init_node('rospy_test', anonymous=True, disable_signals=True)
    # CAREFUL : this should be done only once per PROCESS
    # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.

@nottest
def teardown():

    rospy.signal_shutdown('test complete')

    # finishing all process
    if roscore_process is not None:
        roscore_process.terminate()  # make sure everything is stopped
        while roscore_process.is_alive():
            time.sleep(0.2)  # waiting for roscore to die
        assert not roscore_process.is_alive()


# This simple case works
def test_rospy_unregister_pub():
    """
    Test basic publisher registering/unregistering functionality
    :return:
    """
    setup()
    topicname = '/test/pub_test'

    master = rospy.get_master()

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [p[0] for p in pubs])

    # create the publisher
    pub_test = rospy.Publisher(topicname, std_msgs.Empty, queue_size=1)

    time.sleep(2)  # leave it time
    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname in [p[0] for p in pubs])

    pub_test.unregister()

    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [p[0] for p in pubs])

    teardown()


def fake_sub_cb(data):
    pass


@unittest.expectedFailure
def test_rospy_unregister_sub():
    """
    Test basic subscriber registering/unregistering functionality
    :return:
    """
    setup()
    topicname = '/test/sub_test'

    master = rospy.get_master()

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [s[0] for s in subs])

    # create the publisher
    sub_test = rospy.Subscriber(topicname, std_msgs.Empty, fake_sub_cb)

    time.sleep(2)  # leave it time
    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname in [s[0] for s in subs])  # FAIL ! Topic not registered

    sub_test.unregister()

    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [s[0] for s in subs])
    teardown()


@unittest.expectedFailure
def test_rospy_topic_types():
    """
    Test topic types with registering/unregistering functionality
    :return:
    """
    setup()
    topicname = '/test/pub_type_test'

    master = rospy.get_master()

    pubs, subs, services = master.getSystemState()[2]
    topic_types = master.getTopicTypes()[2]

    assert_true(topicname not in [p[0] for p in pubs])
    assert_true(topicname not in [p[0] for p in topic_types])

    # create the publisher and then try updating again, simulating
    # it coming online after expose call.
    pub_test = rospy.Publisher(topicname, std_msgs.Empty, queue_size=1)

    time.sleep(2)  # leave it time
    pubs, subs, services = master.getSystemState()[2]
    topic_types = master.getTopicTypes()[2]

    assert_true(topicname in [p[0] for p in pubs])
    assert_true(topicname in [p[0] for p in topic_types])

    pub_test.unregister()

    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]
    topic_types = master.getTopicTypes()[2]

    assert_true(topicname not in [p[0] for p in pubs])
    assert_true(topicname not in [p[0] for p in topic_types])  # FAIL ! topic still in types
    teardown()


#### test with both pub and sub

@nottest
def setup_pubsub():
    """
    Test basic publisher registering/unregistering functionality
    :return:
    """
    setup()
    topicname = '/test/pubsub_test'

    master = rospy.get_master()

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [p[0] for p in pubs])
    assert_true(topicname not in [s[0] for s in subs])

    # create the publisher
    pub_test = rospy.Publisher(topicname, std_msgs.Empty, queue_size=1)

    time.sleep(5)  # leave it time
    pubs, subs, services = master.getSystemState()[2]
    print("Publishers :")
    print([p[0] for p in pubs])
    assert_true(topicname in [p[0] for p in pubs])

    # create the subscriber
    sub_test = rospy.Subscriber(topicname, std_msgs.Empty, fake_sub_cb)

    time.sleep(2)  # leave it time
    pubs, subs, services = master.getSystemState()[2]
    print("Subscribers:")
    print([s[0] for s in subs])
    assert_true(topicname in [s[0] for s in subs])

    return topicname, pub_test, sub_test

@nottest
def teardown_pubsub(topic_name):

    master = rospy.get_master()

    pubs, subs, services = master.getSystemState()[2]
    #Note : we know the topic will still be listed in topic types so we don't test for it here
    assert_true(topic_name not in [p[0] for p in pubs])
    assert_true(topic_name not in [s[0] for s in subs])
    teardown()


def test_unregister_pub_first():
    topicname, pub, sub = setup_pubsub()

    master = rospy.get_master()

    pub.unregister()
    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [p[0] for p in pubs])
    assert_true(topicname in [s[0] for s in subs])

    sub.unregister()
    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [p[0] for p in pubs])
    assert_true(topicname not in [s[0] for s in subs])

    teardown_pubsub(topicname)


def test_unregister_sub_first():
    topicname, pub, sub = setup_pubsub()

    master = rospy.get_master()

    sub.unregister()
    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname in [p[0] for p in pubs])
    assert_true(topicname not in [s[0] for s in subs])

    pub.unregister()
    time.sleep(2)  # leave it time

    pubs, subs, services = master.getSystemState()[2]

    assert_true(topicname not in [p[0] for p in pubs])
    assert_true(topicname not in [s[0] for s in subs])

    teardown_pubsub(topicname)


if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()
