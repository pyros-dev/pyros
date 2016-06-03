from __future__ import absolute_import

import os

# This is our default setup to find ROS modules for development, via pyros_setup
# It work out of the box :
# - from a ros devel workspace
# - from a venv for development TODO
# For installed or packaged version, a specific and clear configuration file should be provided.
WORKSPACES = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'devel'),
    os.path.join('/opt', 'yujin', 'amd64', 'indigo-devel')  # required for ConnectionCache message format
]
DISTRO = 'indigo'
