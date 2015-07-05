cmake_minimum_required(VERSION 2.8.3)
project(rostful_node)

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
generate_dynamic_reconfigure_options(cfg/RostfulNode.cfg)
#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_service_files(DIRECTORY srv
    FILES
    StartAction.srv
    CancelAction.srv
    StatusAction.srv
    FeedbackAction.srv
    ResultAction.srv
    StartRapp.srv
    StopRapp.srv
)
generate_messages(DEPENDENCIES std_msgs)

#add_custom_target(rostful_node)  # dummy target
# make sure configure headers are built before any node using them
#add_dependencies(rostful_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(rostful_node ${PROJECT_NAME}_gencfg)

catkin_package( CATKIN_DEPENDS message_runtime std_msgs)

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
