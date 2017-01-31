cmake_minimum_required(VERSION 2.8.3)
project(pyros)

find_package(catkin REQUIRED COMPONENTS
    catkin_pip
    rospy
)

# We need to install the pip dependencies in the workspace being created
# catkin_pip_requirements(${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)
# Currently : no extra dev requirements needed : every required dependency in setup.py

# Forcing to not retrieve all our dependencies, in order to uses system/ROS pacakges.
# TMP : working with pip dependencies
#set (CATKIN_PIP_NO_DEPS False FORCE)

# defining current package as a package that should be managed by pip (not catkin - even though we make it usable with workspaces)
catkin_pip_setup()
catkin_package()

## Unit tests
if (CATKIN_ENABLE_TESTING)


    ##############################################################################
    # Nose Tests
    ##############################################################################

    # Careful this will not support parallel testing, due to multiprocess environment used in tests
    catkin_add_nosetests(pyros/client/tests)
    catkin_add_nosetests(pyros/interfaces/mock/tests)
    catkin_add_nosetests(pyros/tests)

    # TODO : run ROS tests with --no-skip to force them...

endif()
