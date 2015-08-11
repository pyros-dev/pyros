##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

##############################################################################
# Nose Tests
##############################################################################

# catkin_add_nosetests(nosetests)

##############################################################################
# Ros Tests
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)

# Absolute naming test
add_rostest(rosinterface/testStringTopic.test)

# Relative naming test

# Testing rosinterface class integration in rostulf_node
add_rostest(rostful_node/testRosInterface.test)