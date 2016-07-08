Changelog
=========



0.0.99
------

0.1.0 (2016-07-08 - Short Version)
----------------------------------
* Separated Pyros from Rostful. Pyros as generic multiprocess system interface so Rostful remains pure python web REST API, without multiprocess complexity.
* Separated the pure python multiprocess + communication handling into a small package, extracted to pyzmp.
* Build an Abstract Base Class structure to make interfacing more testable, and easier to extend.
* Build a mock interface implementation in order to test and verify basic abstract behaviors.
* Moved the rosinterface into it s special place and wrapped it tightly with tests
* Dropping actions support from Pyros (should be done in another package)
* Added a few methods to do a ros setup dynamically, extracted to pyros-setup
* Now using a configuration file to determine pyros behavior.
* Added feature to use a connection cache to avoid overloading the rosmaster.
* Fixed all tests

0.1.0 (2016-07-08 - Long Version)
---------------------------------
* cleanup debug log.
* moved debug logging to special logdebug file to reduce terminal logspam.
* fixing tests
* disabling some test to prevent catkin test hanging... but test pass when run without --with-xunit. probably a nose issue.
* getting pyzmp 0.0.11 via dependencies to hopefully fix travis.
  not using requirements any longer since we dont have extra dependencies and catkin_pip_setup does install the package in dev mode.
* fixing node behaviors with recent pyzmp.
* reviewing how we use zmp nodes and improving tests... WIP
* fix adding available services.
  improved logging.
  Conflicts:
  pyros/baseinterface/baseinterface.py
  pyros/rosinterface/ros_interface.py
* fixed checking for available transients. now doesnt have to be a dict, just an iterable.
  Conflicts:
  pyros/rosinterface/ros_interface.py
* fix adding available services.
  quick fix on early topics detection to avoid dropping topic interface just after creation. now comparing local topic connection counter with global topic connection counter instead of always assuming 1.
  improved logging.
* fixed checking for available transients. now doesnt have to be a dict, just an iterable.
* next TODO. first step to simplification.
* removed useless None in get(smthg, None)
* added interface cache tests to run by default.
  reverted debug long timeouts.
* finished manual merging of connection_cache_diff_callback.
  fixed all RosInterfaceCache tests, but code really need refactoring...
* more changes from connection_cache_diff_callback branch. only ros_interface.py changes are left todo.
* starting manual merge of connection_cache_diff_callback branch
* fixes for connection cache with diff optimisation.
  added pubsub wait for confirm from cache, but deleted pubsub report deleted before confirmation from cache.
  Not sure if it is the right choice, but extra care is needed when deleting...
* fix tests for RosInterface especially with cache (but no diff optim)
* adding yujin underlay as we need it for connectioncache message format.
* fixing path to current workspace
* now storing endpoints for topics in order to accurately detect lost topics when we get only endpoints diff from cache.
  WIP. some tests breaking now.
* WIP. attempting to fix diff behavior with cache in corner cases when things changing fast on the system.
* changing static method used from class to class method used from self.
* now using diff optimisation in connection_cache
* renaming catkin_pure_python to catkin_pip
* updating for catkin_pure_python 0.1.0
* fixing various minor python issues
* removed duplicated import
* not installing pyros-setup from ROS package. pyros-setup should be useful only if run without ROS (directly from pip).
* fixed service and topic type introspection.
* fixing definitions to match new topic class structure.
* fixing rostest call of testService.py
* fixing self tests. now using pyros_setup pip package.
* locking version numbers for pyros-setup and pyros-test dependencies
* adding nosemain for self test.
* now using pyzmp package dependency instead of internal zmp sources.
  removed submodules
* now travis check python and ros workflows
* moving to package v2
* todo comments. py3 compat.
* replacing obsolete navi/semantic_locations by new /rocon/semantics/locations
* moved pyros and zmp sources, otherwise pyros was not find through egg link.
* added version.
  fixed tests in cmakelists.
  added default config file, removed useless testing config.
  added entry point for selftests.
  added requirements devel dependency to pyros-setup.
* cleaning up rosinterface __init_\_. now doing ros setup only in child node process, dynamically. parent process is isolated.
* cleaning up imports and fixing tests
* refactored to add configuration at module, package and user levels.
  implified pyros-setup configuration from rosinterface.
  reviewed separation between node and interface to isolate all ros setup in child process.
  now doing ROS message conversion internally in rosinterface service and topic classes.
  fixed most tests.
  now uses six to improve python3 compatibility.
* starting to adapt to new configuration from pyros-setup.
* now using catkin_pure_python
* Add Gitter badge
* cosmetics, comments and small fixes...
* readme regarding IoT.
* cosmetics.
* changing reinit method to a setup service.
  now reinitialize rosinterface everytime the list of services or topic passed by the user changes.
  refactor the base interface to hold local copy of system state.
  fix all tests.
* added missing rosservice dependency
* fixing package dependencies for catkin
* fixing catkin build.
* removing unused ROS service specifications
* improved exception handling.
  adding mock client to make unittests easy.
  cosmetics.
* removing dynamic_reconfigure.
* removed rocon feature.
  cleanup
* Improved Readme
* exposing servicecall timeout exception. cosmetics
* fixing log warn -> info for startup args.
* warn -> info when it's not meant to be alarming to the users.
* fixme comments
* adding simple test to assert rospy potentially strange behaviors.
  separating cache and non cache tests.
  catching connection_cache proxy init timeout, showing error and disabling.
* adding custom manager argument in basenode, and making shutdown possible override more obvious.
* ZMP : services and node advertisement now done in context managers.
  Node now support using custom context manager when starting in another process.
  cosmetics.
* improving base support to pass diff instead of query full state everytime.
  now with callback called from connection cache proxy to only process list if change happens.
* fixing reinit to be delayed if ros interface not ready yet.
* fixing pyrosROS test with latest pyros_test
* adding pyrosRos test to catkin tests
* reiniting connection cache if dynamic_reconfigure disable/enable it.
* using enable_cache in dynamic_reconfigure to be able to dynamically switch if needed.
* fixed populating empty message instance. comments.
* adding missing rosnode as test dependency.
* disabling roconinterface dynamic import.
* moving more nodes to pyros-test
* moving nodes to pyros-test.
  skipping tests if connection_cache not found.
* better error message if tests are run from python without pyros-test installed in ROS env.
* using pyros_cfg and fix import in rocont interface, to run nosetests from python venv.
* added generated code for dynamic_reconfigure.
* adding requirements, fixing setup.py for setuptools.
* now allowing to delay the import of rosinterface subpackage and passing base_path to find ROS environment dynamically.
* using ros-shadow-fixed for travis
* cleaning up comments
* adding option to enable cache or not from rosparams.
* ros_interface now using topics and service types from cacche if available, otherwise query one by one when needed.
  making sure cache process is started and stopped during the test to avoid scary harmless warnings.
* improving tests.
* using silent fallback for connectioncache proxy.
* fixing dependencies in package.xml
* pyros now dependein on pyros_setup and pyros_test for tests
* pyros now depending on pyros_setup
* expose_transients_regex now relying on _transient_change_detect directly.
  small refactor to allow transient updates only with ROS system state differences.
  fixing mockinterface to call reinit only after setting up mock
  Added first connection_cache subscriber implementation to avoid pinging the master too often. WIP.
* Contributors: AlexV, Daniel Stonier, The Gitter Badger, alexv

0.0.7
-----
* Now launching node as subprocess to isolate rospy concurrent behavior and other multiprocess libraries.
* Added basic unit tests
* Refactored all components

