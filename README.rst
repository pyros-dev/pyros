# rostful-node
The node embedded in a ROS system to allow rostful to do its job.

This is not meant to be launched by itself.
Instead it should be used as a python package, creating a node for interfacing to the world outside ROS.

Meaning each package depending on rostful-node will create their own ROS node.
It is intended so that the configuration ( what is exposed or not ) can be different for each one of them.
Be careful however in settings where we have multiple similar clients ( like web server scaling ), only one node is needed here.

This node should be launched with roslaunch and follow ros de facto standards, in order to perform tests on it.





