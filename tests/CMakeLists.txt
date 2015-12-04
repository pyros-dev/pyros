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
add_rostest(pyros/rosinterface/testStringTopic.test)
# broken for now

# Relative naming test

# Testing rosinterface class integration in pyros
add_rostest(pyros/testRosInterface.test)