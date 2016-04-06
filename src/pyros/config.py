###
# Generic
###
# Settings to pass to pyros node to interface with another system

TOPICS = []
SERVICES = []
PARAMS = []

###
# ROS specific
###
ROS_USE_CONNECTION_CACHE = False
ROS_CONNECTION_CACHE_LIST_TOPIC = "/rocon/connection_cache/list"
ROS_CONNECTION_CACHE_DIFF_TOPIC = "/rocon/connection_cache/diff"

# Not specifying these means we use pyros.rosinterface default config.
# ROS_SETUP_WORKSPACES = []
# ROS_SETUP_DISTRO = 'indigo'

###
# Mock specific
###
# MOCK_
