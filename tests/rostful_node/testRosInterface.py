#!/usr/bin/env python
import unittest
import sys
import os
import logging
import nose
import collections
import multiprocessing
import time

# trying hard to find our ros code ( different possible run environments )
# We need to support :
# - rostest
# - python -m <package> file
# - nosetests
# - pycharm UI test runs (easy debug)
# - TODO : tox + py.test (benchmark)

try:
    import rosgraph
    import rospy
    import rostopic
    import rostest
    import roslaunch
    from std_msgs.msg import String, Empty
    from rostful_node.rosinterface import RosInterface

except ImportError, ie:
    logging.warn("{exc}".format(exc=ie))

    # If we cannot import, we assume ros environment not setup. We attempt a custom setup for tests here.

    # setting up all python paths
    # CAREFUL : SAME order as setup.bash set the pythonpath
    pythonpath_roscode = collections.OrderedDict({
        'install': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install', 'lib', 'python2.7', 'dist-packages')),
        'devel': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel', 'lib', 'python2.7', 'dist-packages')),
        'indigo': os.path.abspath('/opt/ros/indigo/lib/python2.7/dist-packages'),
        'src': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')),  # should be last since this comes after all other usual ros settings
    })

    for k, p in pythonpath_roscode.iteritems():
        if os.path.exists(p):
            logging.warn("Appending {key} space to python path".format(key=k))
            sys.path.append(p)
            # setting python path needed only to find ros shell commands (rosmaster)
            if p not in os.environ.get("PYTHONPATH", []):
                os.environ["PYTHONPATH"] = p + ':' + os.environ.get("PYTHONPATH",'')

    # This is enough to fix the import. However all rOS environment should be set before importing rospy due to some static behaviors
    # So we do the extra setup here

    # Setting env var like ROS if needed
    # TODO : find the proper place in ros where this is set and use it instead
    indigo_distro_envvar = {
        'ROS_ROOT': '/opt/ros/indigo/share/ros',
        'ROS_PACKAGE_PATH': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')) + ':/opt/ros/indigo/share:/opt/ros/indigo/stacks',
        'ROS_MASTER_URI': 'http://localhost:11311',
        'ROS_DISTRO': 'indigo',
        'ROS_ETC_DIR': '/opt/ros/indigo/etc/ros',
    }
    for k, v in indigo_distro_envvar.iteritems():
        if os.environ.get(k, None) is None:
            os.environ[k] = v

    # setting path to find commands
    ospath_roscode = collections.OrderedDict({
        'install': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install', 'bin')),
        'devel': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel', 'bin')),
        'indigo': os.path.abspath('/opt/ros/indigo/bin'),
    })
    ospath_roscode_reversed = collections.OrderedDict(reversed(list(ospath_roscode.items())))  # because we prepend
    for k, p in ospath_roscode_reversed.iteritems():
        if os.path.exists(p) and p not in os.environ.get("PATH", []):
            logging.warn("Appending {key} space to OS path".format(key=k))
            os.environ["PATH"] = p + ':' +os.environ.get("PATH", '')

    # setting cmake prefix path - rosout needs this
    cmakepath_roscode = collections.OrderedDict({
        'install': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install')),
        'devel': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel')),
        'indigo': os.path.abspath('/opt/ros/indigo'),
    })
    cmakepath_roscode_reversed = collections.OrderedDict(reversed(list(cmakepath_roscode.items())))  # because we prepend
    for k, p in cmakepath_roscode_reversed.iteritems():
        if os.path.exists(p) and p not in os.environ.get("CMAKE_PREFIX_PATH", []):
            logging.warn("Appending {key} space to CMake prefix path".format(key=k))
            os.environ["CMAKE_PREFIX_PATH"] = p + ':' + os.environ.get("CMAKE_PREFIX_PATH", '')

    # setting ldlibrary path - rosout needs this
    ldlibrarypath_roscode = collections.OrderedDict({
        'install': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install', 'lib')),
        'install_arch': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install', 'lib', 'x86_64-linux-gnu')),  # Ref : /opt/ros/indigo/_setup_util.sh
        'devel': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel', 'lib')),
        'devel_arch': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel', 'lib', 'x86_64-linux-gnu')),  # Ref : /opt/ros/indigo/_setup_util.sh
        'indigo': os.path.abspath('/opt/ros/indigo/lib'),
        'indigo_arch': os.path.abspath('/opt/ros/indigo/lib/x86_64-linux-gnu'),  # Ref : /opt/ros/indigo/_setup_util.sh
    })
    ldlibrarypath_roscode_reversed = collections.OrderedDict(reversed(list(ldlibrarypath_roscode.items())))  # because we prepend
    for k, p in ldlibrarypath_roscode_reversed.iteritems():
        if os.path.exists(p) and p not in os.environ.get("LD_LIBRARY_PATH", []):
            logging.warn("Appending {key} space to LD_LIBRARY_PATH".format(key=k))
            os.environ["LD_LIBRARY_PATH"] = p + ':' + os.environ.get("LD_LIBRARY_PATH", '')

    import rosgraph
    import rospy
    import rostopic
    import rostest
    import roslaunch
    from std_msgs.msg import String, Empty
    from rostful_node.rosinterface import RosInterface


rostest_enabled = False  # default for python or nose runs

roscore_process = None  # default : we dont need to run roscore
# test node process not setup by default (rostest dont need it here)
empty_srv_process = None
trigger_srv_process = None


# This should have the same effect as the <name>.test file for rostest. Should be used only by nose ( or other python test tool )
def setup_module():
    if not rostest_enabled:
        if not rosgraph.masterapi.is_online():
            global roscore_process
            # Trying to solve this : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
            def ros_core_launch():
                roslaunch.main(['roscore', '--core'])  # same as rostest_main implementation

            roscore_process = multiprocessing.Process(target=ros_core_launch)
            roscore_process.start()

        # Workaround until https://github.com/ros/ros_comm/pull/711 is merged and released
        time.sleep(2)

        # Start roslaunch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # start required nodes - needs to matcht he content of *.test files for rostest to match
        global empty_srv_process, trigger_srv_process
        empty_srv_node = roslaunch.core.Node('pyros', 'emptyService.py', name='empty_service')
        trigger_srv_node = roslaunch.core.Node('pyros', 'triggerService.py', name='trigger_service')
        empty_srv_process = launch.launch(empty_srv_node)
        trigger_srv_process = launch.launch(trigger_srv_node)


def teardown_module():
    if not rostest_enabled:
        # finishing all process are finished
        if empty_srv_process is not None:
            empty_srv_process.stop()
        if trigger_srv_process is not None:
            trigger_srv_process.stop()
        if roscore_process is not None:
            roscore_process.terminate()  # make sure everything is stopped
        # in case we dont shutdown with roscore stop
        rospy.signal_shutdown('test complete')


class TestRosInterface(unittest.TestCase):
    def setUp(self):
        rospy.init_node('ros_interface_test')
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)
        
        self.interface = RosInterface(run_watcher=False)

    def tearDown(self):
        self.interface = None

    ##
    # Test basic topic adding functionality for a topic which already exists in
    # the ros environment.
    def test_topic_add_basic_existing(self):
        topicname = '/test/string'
        self.interface.add_topic(topicname)
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # added a single instance of the topic, so the number of the topics
        # existing should be incremented by one (because we consider the fact
        # that when the entire system is running, two additional topics will be
        # created by the subscriber proxy)
        self.assertEqual(-1, self.interface.topics_args[topicname])
        # the topic already exists in the environment, so it should not be in
        # the waiting list
        self.assertTrue(topicname not in self.interface.topics_waiting)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.topics)

    ##
    # Test basic topic adding functionality for a topic which does not yet exist
    # in the ros environment
    def test_topic_add_basic_nonexistent(self):
        topicname = '/test/nonexistent'
        self.interface.add_topic(topicname)
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # we added the topic, but it didn't exist yet, so the count should be -2
        self.assertEqual(-2, self.interface.topics_args[topicname])
        # topic does not exist in the environment, so it should be in the waiting list
        self.assertTrue(topicname in self.interface.topics_waiting)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics)

        # create the publisher and then try adding the topic again, simulating
        # it coming online.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)
        self.interface.add_topic(topicname)

        self.assertTrue(topicname in self.interface.topics_args)
        self.assertEqual(-1, self.interface.topics_args[topicname])
        self.assertTrue(topicname in self.interface.topics)
        self.assertTrue(topicname not in self.interface.topics_waiting)

    @unittest.expectedFailure
    def test_topic_del(self):
        print("topic del")
        self.assertTrue(False)

    @unittest.expectedFailure
    def test_service_add(self):
        print("service del")
        self.assertTrue(False)

    @unittest.expectedFailure
    def test_service_del(self):
        print("service del")
        self.assertTrue(False)

    ##
    # Ensure that no crash occurs when the expose topics function is called by a
    # reconfigure request which requires the deletion of some topics from the
    # topic arguments that already exist.
    def test_expose_topics_deletion_crash(self):
        self.interface.add_topic('/test/string')
        self.interface.add_topic('/test/empty')
        
        self.interface.expose_topics(['/test/string'])
        # ensure all the lists/dicts no longer contain the deleted topic
        self.assertTrue('/test/empty' not in self.interface.topics_args)
        self.assertTrue('/test/empty' not in self.interface.topics)
        self.assertTrue('/test/empty' not in self.interface.topics_waiting)
        # ensure all relevant lists have the remaining one
        self.assertTrue('/test/string' in self.interface.topics_args)
        self.assertTrue('/test/string' in self.interface.topics)

    ##
    # Ensure that no crash occurs when the expose services function is called by
    # a reconfigure request which requires the deletion of some services from the
    # service arguments that already exist.
    def test_expose_services_deletion_crash(self):
        self.interface.add_service('/test/empsrv')
        self.interface.add_service('/test/trgsrv')
        self.interface.expose_services(['/test/empsrv'])
        # ensure all the lists/dicts no longer contain the deleted service
        self.assertTrue('/test/trgsrv' not in self.interface.services_args)
        self.assertTrue('/test/trgsrv' not in self.interface.services)
        self.assertTrue('/test/trgsrv' not in self.interface.services_waiting)

        # ensure all relevant lists have the remaining one
        self.assertTrue('/test/empsrv' in self.interface.services_args)
        self.assertTrue('/test/empsrv' in self.interface.services)

    # def test_reconfigure_topic(self):
    #     config = {'services': [], 'actions': []}
    #     config['topics'] = "['/test/1', '/test/2', '/test/3', '/testreg/.*', '/.*/regtest']"
    #     self.interface.reconfigure(config, 1)
    #     expected_dict = {'/test/1': -2, '/test/2': -2, '/test/3': -2, '/testreg/.*': -2, '/.*/regtest': -2}
    #     expected_list = ['/test/1', '/test/2', '/test/3', '/testreg/.*', '/.*/regtest']
    #     self.assertEqual(self.interface.topics_args, expected_dict)
    #     # don't care about ordering, just contents
    #     self.assertEqual(sorted(self.interface.topics_waiting), sorted(expected_list))

    # @unittest.expectedFailure
    # def test_reconfigure_service(self):
    #     print("recon service")

    # ##
    # # Ensure that if there are malformed strings in the reconfigure request they are ignored
    # def test_reconfigure_malformed(self):
    #     config = {'actions': []}
    #     config['topics'] = "['/test(/1bad', '/test/)bad', '/test/good']"
    #     config['services'] = "['/test(/1bad', '/test/)bad', '/test/good']"
    #     self.interface.reconfigure(config, 1)
    #     assert self.interface.topics_args == {}
    #     assert self.interface.services_args == []
        
if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )

    # Ros arguments will tell us if we started from ros, or from straight python
    rosargs = [arg for arg in sys.argv if arg.startswith("__")]

    if len(rosargs) > 0 :
        rostest_enabled = True
        rostest.rosrun('test_ros_interface', 'test_all', TestRosInterface)
    else :
        #import nose
        nose.runmodule()
