cmake_minimum_required(VERSION 2.8.3)
project(pyros)

find_package(catkin REQUIRED COMPONENTS
    catkin_pip
    rospy
)

# We need to install the pip dependencies in the workspace being created
# catkin_pip_requirements(${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)
# Currently : no extra dev requirements needed : every required dependency in setup.py

# defining current package as a package that should be managed by pip (not catkin - even though we make it usable with workspaces)
catkin_pip_setup()
catkin_package()

## Unit tests
if (CATKIN_ENABLE_TESTING)


    ##############################################################################
    # Nose Tests
    ##############################################################################

    # Careful this will not support parallel testing, due to multiprocess environment used in tests
    catkin_add_nosetests(pyros/mockinterface/tests)
    # because of ROS (node_init() can be called only once per process)
    # these should be run in separate process each to avoid global side effect during tests
    # TODO : find a way to force nose to run each test isolated ?
    #catkin_add_nosetests(pyros/rosinterface/tests)
    catkin_add_nosetests(pyros/tests)

    ##############################################################################
    # Ros Tests
    ##############################################################################

    # find_package(catkin REQUIRED COMPONENTS rostest)

    # These rostests need to be run one by one ( because of rospy side effects)
    # Note : using pyros-setup should fix this...
    catkin_add_nosetests(pyros/rosinterface/rostests/testService.py)
    catkin_add_nosetests(pyros/rosinterface/rostests/testStringTopic.py)
    catkin_add_nosetests(pyros/rosinterface/rostests/testRosInterface.py)
    catkin_add_nosetests(pyros/rosinterface/rostests/testRosInterfaceCache.py)
    # Running with nose because rostest fails ( not sure why yet )

    # add_rostest(pyros/rosinterface/rostests/testStringTopic.test)
    # add_rostest(pyros/rosinterface/rostests/testService.test)

    # Testing rosinterface class integration in pyros
    # add_rostest(pyros/rosinterface/rostests/testRosInterface.test)


endif()
