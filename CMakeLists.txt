cmake_minimum_required(VERSION 2.8.3)
project(pyros)

find_package(catkin REQUIRED COMPONENTS
    catkin_pure_python
    rospy
)

# We need to install the pip dependencies in the workspace being created
catkin_pip_requirements(${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)

# defining current package as a package that should be managed by pip (not catkin - even though we make it usable with workspaces)
catkin_pip_package()

## Unit tests
if (CATKIN_ENABLE_TESTING)


    ##############################################################################
    # Nose Tests
    ##############################################################################

    #catkin_add_nosetests(src/zmp/tests)  # this makes nose hang with --with-xunit sometimes. no idea why.
    # One by one instead
    catkin_add_nosetests(src/zmp/tests/test_message.py)
    catkin_add_nosetests(src/zmp/tests/test_node.py)
    catkin_add_nosetests(src/zmp/tests/test_service.py)

    # Careful this will not support parallel testing, due to multiprocess environment used in tests
    catkin_add_nosetests(src/pyros/mockinterface/tests)
    # because of ROS these should be run in separate process each to avoid global side effect during tests
    #catkin_add_nosetests(src/pyros/rosinterface/tests)
    catkin_add_nosetests(src/pyros/tests)

    ##############################################################################
    # Ros Tests
    ##############################################################################

    # find_package(catkin REQUIRED COMPONENTS rostest)

    # These rostests need to be run one by one ( because of rospy side effects)
    # Note : using pyros-setup should fix this...
    catkin_add_nosetests(src/pyros/rosinterface/rostests/testService.py)
    catkin_add_nosetests(src/pyros/rosinterface/rostests/testStringTopic.py)
    catkin_add_nosetests(src/pyros/rosinterface/rostests/testRosInterface.py)
    catkin_add_nosetests(src/pyros/rosinterface/rostests/testPyrosROS.py)
    # Running with nose because rostest fails ( not sure why yet )

    # add_rostest(src/pyros/rosinterface/rostests/testStringTopic.test)
    # add_rostest(src/pyros/rosinterface/rostests/testService.test)

    # Testing rosinterface class integration in pyros
    # add_rostest(src/pyros/rosinterface/rostests/testRosInterface.test)


endif()
