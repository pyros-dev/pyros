#!/usr/bin/env python
# A very special ROS hack that emulate a ros environment when imported from python
# Useful for using all python tools ( tests, IDE, etc. ) without having to do all the ROS setup beforehand
from __future__ import absolute_import

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

except ImportError, ie:
    logging.warn("{exc}".format(exc=ie))
    logging.warn("ROS environment not detected. Emulating setup now...")

    # If we cannot import, we assume ros environment not setup. We attempt a custom setup for tests here.

    # setting up all python paths
    # CAREFUL : SAME order as setup.bash set the pythonpath
    # TODO : better way : use CMAKE_PREFIX_PATH and get_workspace() to get that dynamically ?
    pythonpath_roscode = collections.OrderedDict({
        'install': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'install', 'lib', 'python2.7', 'dist-packages')),
        'devel': os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel', 'lib', 'python2.7', 'dist-packages')),
        'indigo': os.path.abspath('/opt/ros/indigo/lib/python2.7/dist-packages'),
    })

    for k, p in pythonpath_roscode.iteritems():
        if os.path.exists(p):
            logging.warn("Appending {key} space to python path".format(key=k))
            sys.path.append(p)
            # setting python path needed only to find ros shell commands (rosmaster)
            if p not in os.environ.get("PYTHONPATH", []):
                os.environ["PYTHONPATH"] = p + ':' + os.environ.get("PYTHONPATH", '')

    # This is enough to fix the import.
    # However all ROS environment should be set before importing rospy due to https://github.com/ros/catkin/issues/767
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

    logging.warn("ROS environment emulating setup done.")


rostest_enabled = False  # default for python or nose runs

roscore_process = None  # default : we dont need to run roscore


# This should have the same effect as launching the <name>.test file for rostest ( before interpreting the contents )
# Should be used only by nose ( or other python test tool )
def rostest_nose_setup_module():
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


def rostest_nose_teardown_module():
    if not rostest_enabled:
        # finishing all process
        if roscore_process is not None:
            roscore_process.terminate()  # make sure everything is stopped
        # in case we dont shutdown with roscore stop
        rospy.signal_shutdown('test complete')


def is_rostest_enabled():
    return rostest_enabled


def rostest_or_nose_main(package, test_name, test, sysargv):
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )

    # Ros arguments will tell us if we started from ros, or from straight python
    rosargs = [arg for arg in sys.argv if arg.startswith("__")]

    if len(rosargs) > 0:
        global rostest_enabled
        rostest_enabled = True
        rostest.rosrun(package, test_name, test, sysargv)
    else:
        nose.runmodule()
