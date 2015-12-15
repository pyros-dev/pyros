
# A very special ROS hack that emulate a ros environment when imported from python
# Useful for using all python tools ( tests, IDE, etc. ) without having to do all the ROS setup beforehand

import sys
import os
import logging
import collections

# Functions needed to find our ros code from different possible run environments,
#  even when ROS has not been setup previously ( source setup, roslaunch, rostest ).
# This is especially useful when debugging directly from Python IDE or so.
#
# Note : We dont want to support all the overlays / underlay things here
# -> only applicable for package on top of core ROS packages.
#
# Here we need to setup an environment that can support different types of run:
#
# - nosetests
# - pycharm UI test runs (with easy debug)
# - rostest  => not supported yet
#    rostest try to run the python code as executable -> needs more hacks to get modules from source
#    nosetest has special import code that discover the package modules nicely
#    Still needs more investigation in why rostest doesnt work / works differently to nose.
#    Maybe not worth it.
# - TODO : tox + py.test (benchmark)

def ROS_setup_rosdistro_env(default_distro=None):

    # TODO : investigate if running the actual setup.bash script (from proper workspace) wouldn't be better :
    # import os
    # import sys
    # import pprint
    # import subprocess
    #
    # command = ['bash', '-c', 'source /opt/ros/indigo/setup.bash && env']
    #
    # proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    #
    # for line in proc.stdout:
    #   (key, _, value) = line.partition("=")
    #   os.environ[key] = value.rstrip()
    #   if key == 'PYTHONPATH':
    #       for newp in [p for p in value.split(':') if p not in sys.path]:
    #           sys.path.append(newp)
    #
    # proc.communicate()

    default_distro = default_distro or 'indigo'
    # Setting env var like ROS would
    # TODO : find the proper place in ros where this is set and use it instead
    if os.environ.get('ROS_DISTRO', None) is None:
                os.environ['ROS_DISTRO'] = default_distro

    distro = os.environ['ROS_DISTRO']
    if os.environ.get('ROS_ROOT', None) is None:
                os.environ['ROS_ROOT'] = '/opt/ros/' + distro + '/share/ros'

    if os.environ.get('ROS_PACKAGE_PATH', None) is None:
                os.environ['ROS_PACKAGE_PATH'] = ':'.join(['/opt/ros/' + distro + '/share',
                                                           '/opt/ros/' + distro + '/stacks'])

    if os.environ.get('ROS_MASTER_URI', None) is None:
                os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'

    if os.environ.get('ROS_MASTER_URI', None) is None:
                os.environ['ROS_MASTER_URI'] = '/opt/ros/' + distro + '/etc/ros'


def ROS_find_workspaces(cmake_env_var=None):
    cmake_env_var = cmake_env_var or "CMAKE_PREFIX_PATH"
    try:
        workspace_paths = os.environ[cmake_env_var].split(':')
        install_ws = workspace_paths[0]
        devel_ws = workspace_paths[1]
        src_path = devel_ws + "/../src"
        distro_path = workspace_paths[2]

    # TODO : more accurate error detection
    except Exception:
        install_ws = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'install'))
        devel_ws = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'devel'))
        src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'src'))
        distro_path = os.path.abspath('/opt/ros/indigo')
        # setting cmake prefix path - rosout needs this
        for k, p in zip(['distro', 'devel', 'install'], [distro_path, devel_ws, install_ws]):
            if os.path.exists(p) and p not in os.environ.get("CMAKE_PREFIX_PATH", []):
                logging.warn("Appending {key} space to CMake prefix path".format(key=k))
                os.environ["CMAKE_PREFIX_PATH"] = p + ':' + os.environ.get("CMAKE_PREFIX_PATH", '')

    # prepending current path for ros package discovery
    os.environ['ROS_PACKAGE_PATH'] = src_path + ':' + os.environ['ROS_PACKAGE_PATH']

    return distro_path, install_ws, devel_ws


def ROS_setup_ospath(distro_space=None, install_workspace=None, devel_workspace=None):

    distro_space = distro_space or ROS_find_workspaces()[0]  # we want to enforce distro workspace
    install_workspace = install_workspace  # we don't want to enforce install or devel workspace
    devel_workspace = devel_workspace

    # setting path to find commands
    ospath_roscode = collections.OrderedDict({
        'install': os.path.join(install_workspace, 'bin') if install_workspace else None,
        'devel': os.path.join(devel_workspace, 'bin') if devel_workspace else None,
        'indigo': os.path.join(distro_space, 'bin'),
    })
    ospath_roscode_reversed = collections.OrderedDict(reversed(list(ospath_roscode.items())))  # because we prepend
    for k, p in ospath_roscode_reversed.iteritems():
        if p is not None and os.path.exists(p) and p not in os.environ.get("PATH", []):
            logging.warn("Appending {key} space to OS path".format(key=k))
            os.environ["PATH"] = p + ':' + os.environ.get("PATH", '')

    # setting ldlibrary path - rosout needs this
    ldlibrarypath_roscode = collections.OrderedDict({
        'install': os.path.join(install_workspace, 'lib') if install_workspace else None,
        'install_arch': os.path.join(install_workspace, 'lib', 'x86_64-linux-gnu') if install_workspace else None,  # Ref : /opt/ros/indigo/_setup_util.sh
        'devel': os.path.join(devel_workspace, 'lib') if devel_workspace else None,
        'devel_arch': os.path.join(devel_workspace, 'lib', 'x86_64-linux-gnu')if devel_workspace else None,  # Ref : /opt/ros/indigo/_setup_util.sh
        'indigo': os.path.join(distro_space, 'lib'),
        'indigo_arch': os.path.join(distro_space, 'lib/x86_64-linux-gnu'),  # Ref : /opt/ros/indigo/_setup_util.sh
    })
    ldlibrarypath_roscode_reversed = collections.OrderedDict(reversed(list(ldlibrarypath_roscode.items())))  # because we prepend
    for k, p in ldlibrarypath_roscode_reversed.iteritems():
        if p is not None and os.path.exists(p) and p not in os.environ.get("LD_LIBRARY_PATH", []):
            logging.warn("Appending {key} space to LD_LIBRARY_PATH".format(key=k))
            os.environ["LD_LIBRARY_PATH"] = p + ':' + os.environ.get("LD_LIBRARY_PATH", '')


# TODO : check if we can use roslib.load_manifest for all this
def ROS_setup_pythonpath(distro_space=None, install_workspace=None, devel_workspace=None):

    distro_space = distro_space or ROS_find_workspaces()[0]
    install_workspace = install_workspace
    devel_workspace = devel_workspace

    # setting up all python paths
    # CAREFUL : SAME order as setup.bash set the pythonpath
    pythonpath_roscode = collections.OrderedDict({
        'install': os.path.join(install_workspace, 'lib', 'python2.7', 'dist-packages') if install_workspace else None,
        'devel': os.path.join(devel_workspace, 'lib', 'python2.7', 'dist-packages') if devel_workspace else None,
        'indigo': os.path.join(distro_space, 'lib', 'python2.7', 'dist-packages'),
    })

    for k, p in pythonpath_roscode.iteritems():
        if p is not None and os.path.exists(p):
            logging.warn("Appending {key} space to python path".format(key=k))
            sys.path.append(p)
    pythonpath_roscode_reversed = collections.OrderedDict(reversed(list(pythonpath_roscode.items())))  # because we prepend
    for k, p in pythonpath_roscode_reversed.iteritems():
            # setting python path needed only to find ros shell commands (rosmaster)
            if p is not None and p not in os.environ.get("PYTHONPATH", []):
                os.environ["PYTHONPATH"] = p + ':' + os.environ.get("PYTHONPATH", '')

    # This is enough to fix the import.
    # However all ROS environment should be setup before importing rospy due to https://github.com/ros/catkin/issues/767


def ROS_emulate_setup(distro='indigo', prio_install=False):  # by default we like the devel workspace
    logging.warn(" => Emulating ROS setup now...")
    ROS_setup_rosdistro_env(default_distro=distro)
    distro_ws, install_ws, devel_ws = ROS_find_workspaces()
    ROS_setup_ospath(distro_ws, install_ws if prio_install else None, devel_ws)
    ROS_setup_pythonpath(distro_ws, install_ws if prio_install else None, devel_ws)
    logging.warn(" => ROS setup emulation done.")
