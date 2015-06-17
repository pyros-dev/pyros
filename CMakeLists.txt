cmake_minimum_required(VERSION 2.8.3)
project(rostful_node)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint rospy actionlib_msgs dynamic_reconfigure)
catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/rostful-node.cfg)

# make sure configure headers are built before any node using them
#add_dependencies(rostful-node ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_package()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
