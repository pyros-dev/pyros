cmake_minimum_required(VERSION 2.8.3)
project(pyros)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS
    roslint
    rospy
    actionlib_msgs
    dynamic_reconfigure
    std_msgs
    message_generation
)
catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/Pyros.cfg)
#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_service_files(DIRECTORY srv
    FILES
    StartRapp.srv
    StopRapp.srv
    StringEchoService.srv
)
generate_messages(DEPENDENCIES std_msgs)

#add_custom_target(pyros)  # dummy target
# make sure configure headers are built before any node using them
#add_dependencies(pyros ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(pyros ${PROJECT_NAME}_gencfg)

catkin_package( CATKIN_DEPENDS message_runtime std_msgs)

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})


# Trick to get catkin generated classes imported by nose and other usual python tools

set(link "${CMAKE_CURRENT_SOURCE_DIR}/src/pyros/srv")
set(path "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/srv")

if(NOT EXISTS "${link}" AND EXISTS "${path}")
    message(STATUS "linking catkin generated classes from ${path} to source space in ${link}")
    execute_process(
      # Note : return code is broken in 2.8 (trusty version) https://cmake.org/Bug/view.php?id=14928
      COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" "${path}" "${link}"
    )
endif()

set(link "${CMAKE_CURRENT_SOURCE_DIR}/src/pyros/cfg")
set(path "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg")

if(NOT EXISTS "${link}" AND EXISTS "${path}")
    message(STATUS "linking catkin generated classes from ${path} to source space in ${link}")
    execute_process(
      # Note : return code is broken in 2.8 (trusty version) https://cmake.org/Bug/view.php?id=14928
      COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" "${path}" "${link}"
    )
endif()

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
