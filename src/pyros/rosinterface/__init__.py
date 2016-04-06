# -*- coding: utf-8 -*-
#This python package is handling all ROS related communication for rostful-node.
from __future__ import absolute_import

import os
import types
import sys

"""
Hopefully this should endup in rosinterface.__doc__
"""

# This should be found (from python dependencies at least)
import pyros_setup

from . import pyros_setup_config

# Here we use pyros_setup configuration to setup ROS if needed before using any of our modules
# This is much simpler since we don't need any other configuration at import time.
# Extra Pyros configuration can be passed at runtime.

try:
    import rospy  # early except to prevent unintentional workaround in all modules here
    from pyros.rosinterface.topic import TopicBack
    from pyros.rosinterface.service import ServiceBack
    from pyros.rosinterface.param import ParamBack
    from pyros.rosinterface.ros_interface import RosInterface
except ImportError:
    # This will load the pyros_setup configuration from a local object
    pyros_setup.configurable_import(instance_relative_config=False).configure(pyros_setup_config).activate()
    import rospy
    from pyros.rosinterface.topic import TopicBack
    from pyros.rosinterface.service import ServiceBack
    from pyros.rosinterface.param import ParamBack
    from pyros.rosinterface.ros_interface import RosInterface

# BUT for pyros we want to be able to pass these as a part of pyros configuration
# This is done by the root package modules before they import this rosinterface module

# # Design duplicated from pyros-setup.__init__.py
# # TODO : find a better way to reuse code...
#
# # class to allow delayed conditional import, with behavior based on configuration.
# # This way it can work with or without preset environment
# class _RosInterfaceSetup(pyros_setup.ConfigImport):
#     #: Default configuration parameters.
#     _default_config = {
#         'DISTRO': 'indigo',
#         'WORKSPACES': [],
#     }
#
#     def __init__(self, instance_path=None, instance_relative_config=True, root_path=None):
#
#         relay_import_dict = {
#             'rospy': 'rospy',  # early except to detect errors and prevent unintentional useless workarounds
#             'ros_utils': ('.ros_utils', __package__),
#         }
#         fix_imports = self._attempt_import_fix
#
#         super(_PyrosSetup, self).__init__('pyros_setup',
#                                           'Small setup package to dynamically interface with ROS',
#                                           relay_import_dict=relay_import_dict,
#                                           fix_imports=fix_imports,
#                                           instance_path=instance_path,
#                                           instance_relative_config=instance_relative_config,
#                                           root_path=root_path,
#                                           default_config=self._default_config)
#
#     def _attempt_import_fix(self):
#         from .ros_setup import ROS_emulate_setup
#         # we want this to expect in case of bad config, because default_config has to have these fields.
#         ROS_emulate_setup(self.config['DISTRO'], *self.config['WORKSPACES'])
#
#     def activate(self):
#         super(_PyrosSetup, self).activate()
#
#         # defining shortcuts after successful import
#         self.get_master = self.ros_utils.get_master
#         self.get_ros_home = self.ros_utils.get_ros_home
#
#         return self
#
#
#
# class _PyrosSetup(types.ModuleType):
#
#     #: Default configuration parameters.
#     default_config = {
#         # pretty standard, but actually useful ?
#         'DEBUG': False,
#         'TESTING': False,
#         # required
#         'PYROS_SETUP_DISTRO': 'indigo',
#         'PYROS_SETUP_WORKSPACES': [],
#     }
#
#     def __init__(self, import_name=None, instance_path=None, instance_relative_config=True, root_path=None):
#         super(_PyrosSetup, self).__init__('RosInterface', 'Dynamically generated module to interface with ROS')
#
#         import_name = import_name or 'RosInterface'
#
#         self.config_handler = pyros_setup.config.PackageBoundConfigHandler(
#             import_name,
#             instance_path=instance_path,
#             instance_relative_config=instance_relative_config,
#             root_path=root_path,
#             default_config=self.default_config
#         )
#
#     def activate(self):
#         # TODO : use imp module, or some more fancy stuff (PyMacro style implementation)
#         # TODO : put that in context to allow deactivation...
#         # The actual trick
#         try:
#             import rospy  # early except to prevent unintentional workaround in all modules here
#             from pyros.rosinterface.topic import TopicBack
#             from pyros.rosinterface.service import ServiceBack
#             from pyros.rosinterface.param import ParamBack
#             from pyros.rosinterface.ros_interface import RosInterface
#             from pyros.rosinterface.pyros_ros import PyrosROS
#         except ImportError:
#
#             # Here we expect a part of the configuration to be destined to pyros_setup
#             # So we create a similar handler and pass only a subset of the config.
#             pyros_setup\
#                 .delayed_import_new(instance_path=self.instance_path, instance_relative_config=self.instance_relative_config, root_path=self.root_path)\
#                 .configure_from_mappings(**self.config.get_namespace('PYROS_SETUP_', lowercase=False))\
#                 .activate()
#             import rospy
#             from pyros.rosinterface.topic import TopicBack
#             from pyros.rosinterface.service import ServiceBack
#             from pyros.rosinterface.param import ParamBack
#             from pyros.rosinterface.ros_interface import RosInterface
#             from pyros.rosinterface.pyros_ros import PyrosROS
#
#         # members simulating a usual imported module
#         self.TopicBack = TopicBack
#         self.ServiceBack = ServiceBack
#         self.ParamBack = ParamBack
#         self.RosInterface = RosInterface
#         self.PyrosROS = PyrosROS
#         # subpackages
#         # Is it really needed ? big risk of circular imports here...
#         #self.rostests = __import__("pyros.rosinterface.rostests", fromlist=["pyros.rosinterface"])
#         #self.tests = __import__("pyros.rosinterface.tests", fromlist=["pyros.rosinterface"])
#         # submodules for individual access
#         self.message_conversion = __import__("pyros.rosinterface.message_conversion", fromlist=["pyros.rosinterface"])
#         # TODO : we shouldnt need it...
#
#         # CAREFUL this doesn't work sometimes (had problem when using from celery bootstep...)
#         sys.modules['pyros_setup'] = self
#         return self
#
#
#     # encapsulating local imports to delay them until ROS setup is done
#     @classmethod
#     @pyros_setup.deprecated
#     def delayed_import(cls, distro=None, *workspaces):
#
#         # building a one time config for bwcompat purpose
#         cfg = cls.default_config
#         cfg['DISTRO'] = distro or 'indigo'
#         cfg['WORKSPACES'] = workspaces
#         module_redirect = cls()
#         module_redirect.configure_from_object(cfg)
#         module_redirect.activate()
#
#         return module_redirect
#
#     @classmethod
#     @pyros_setup.deprecated
#     def delayed_import_auto(cls, distro=None, base_path=None):
#         import os
#
#         distro = distro or 'indigo'
#         # default basepath working if pyros-setup is directly cloned in your workspace
#         # This file think it is in devel/lib/python2.7/dist-packages/pyros_setup for some reason...
#         # NOTE : maybe the default here is a bad idea, making it simpler than it should be for the user...
#         base_path = base_path or os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')
#
#         # using this for bwcompat purpose only. now seems like a bad idea after all.
#         from pyros_setup.ros_setup import ROS_find_workspaces
#         workspaces = ROS_find_workspaces(distro, base_path)
#
#         return cls.delayed_import()
#
#     #
#     # Factory method
#     #
#
#     @classmethod
#     def configurable_import(cls, instance_path=None, instance_relative_config=True, root_path=None):
#         # we return a relay of imported names, accessible the same way a direct import would be.
#         module_redirect = cls(
#             instance_path=instance_path,
#             instance_relative_config=instance_relative_config,
#             root_path=root_path
#         )
#         return module_redirect
#
#
# delayed_import = _PyrosSetup.delayed_import
# delayed_import_auto = _PyrosSetup.delayed_import_auto
# configurable_import = _PyrosSetup.configurable_import
#
# __all__ = [
#     'delayed_import',
#     'delayed_import_auto',
#     'delayed_import_new',
# ]

__all__ = [
    'TopicBack',
    'ServiceBack',
    'ParamBack',
    'RosInterface',
]