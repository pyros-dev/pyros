^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyros
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2016-08-25)
------------------
* Merge pull request `#93 <https://github.com/asmodehn/pyros/issues/93>`_ from asmodehn/new_six
  removing old gone six submodule
* removing old gone six submodule
* cleanup debug log.
* Merge pull request `#84 <https://github.com/asmodehn/pyros/issues/84>`_ from asmodehn/connection_cache_diff_callback
  Connection cache diff callback
* moved debug logging to special logdebug file to reduce terminal logspam.
* fixing tests
* fixed check for early topic detection and stabilize cache diff optimization.
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
* Merge branch 'indigo-devel' of https://github.com/asmodehn/pyros into config_refactor
  Conflicts:
  package.xml
  pyros/pyros_client.py
* removed duplicated import
* not installing pyros-setup from ROS package. pyros-setup should be useful only if run without ROS (directly from pip).
* fixed service and topic type introspection.
* fixing definitions to match new topic class structure.
* fixing rostest call of testService.py
* fixing self tests. now using pyros_setup pip package.
* locking version numbers for pyros-setup and pyros-test dependencies
* Merge branch 'cmake_buildtool' of https://github.com/asmodehn/pyros into config_refactor
  Conflicts:
  CMakeLists.txt
  pyros/pyros_mock.py
  requirements.txt
  setup.py
  src/pyros/baseinterface/basenode.py
  src/pyros/rosinterface/pyros_ros.py
  src/pyros/tests/testPyrosMock.py
  zmp/__init_\_.py
  zmp/exceptions.py
  zmp/master.py
  zmp/message.py
  zmp/node.py
  zmp/proto/service.proto
  zmp/service.py
  zmp/service_pb2.py
  zmp/tests/profile_node.py
  zmp/tests/test_message.py
  zmp/tests/test_node.py
  zmp/tests/test_service.py
  zmp/topic.py
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
* Merge pull request `#71 <https://github.com/asmodehn/pyros/issues/71>`_ from gitter-badger/gitter-badge
  Add a Gitter chat badge to README.rst
* Add Gitter badge
* Merge pull request `#69 <https://github.com/asmodehn/pyros/issues/69>`_ from asmodehn/multiclient
  cosmetics, comments and small fixes...
* cosmetics, comments and small fixes...
* Merge pull request `#62 <https://github.com/asmodehn/pyros/issues/62>`_ from asmodehn/remove_unused_features
  Remove unused features
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
* Merge branch 'improve_exceptions_handling' into remove_unused_features
* improved exception handling.
  adding mock client to make unittests easy.
  cosmetics.
* removing dynamic_reconfigure.
* removed rocon feature.
  cleanup
* Improved Readme
* exposing servicecall timeout exception. cosmetics
* Merge pull request `#55 <https://github.com/asmodehn/pyros/issues/55>`_ from stonier/verbosity
  Logging : WARN -> INFO
* fixing log warn -> info for startup args.
* warn -> info when it's not meant to be alarming to the users.
* fixme comments
* Merge commit '971199c' into indigo-devel
* Merge commit '15aab53' into indigo-devel
* Merge pull request `#48 <https://github.com/asmodehn/pyros/issues/48>`_ from asmodehn/connection_cache
  Connection cache
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
* Merge branch 'strict-python-exp' into connection_cache
* using enable_cache in dynamic_reconfigure to be able to dynamically switch if needed.
* fixed populating empty message instance. comments.
* Merge pull request `#50 <https://github.com/asmodehn/pyros/issues/50>`_ from asmodehn/strict-python-exp
  Strict python experiment
* Merge branch 'connection_cache' of https://github.com/asmodehn/pyros into strict-python-exp
* adding missing rosnode as test dependency.
* disabling roconinterface dynamic import.
* moving more nodes to pyros-test
* moving nodes to pyros-test.
  skipping tests if connection_cache not found.
* better error message if tests are run from python without pyros-test installed in ROS env.
* using pyros_cfg and fix import in rocont interface, to run nosetests from python venv.
* added generated code for dynamic_reconfigure.
* adding requirements, fixing setup.py for setuptools.
* Merge pull request `#49 <https://github.com/asmodehn/pyros/issues/49>`_ from asmodehn/pyros_setup_fixes
  now allowing to delay the import of rosinterface subpackage and passi…
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

0.0.8 (2016-01-25)
------------------
* doing zmp tests one by one to workaround nose hanging bug with option --with-xunit
* Merge pull request `#45 <https://github.com/asmodehn/pyros/issues/45>`_ from asmodehn/update_timed
  ZMP node now passing timedelta to update.
* making service and param new style classes.
* fixing throttling to reinitialize last_update in basenode.
* fixing a few quantifiedcode issues...
* ZMP node now passing timedelta to update.
  Pyros nodes now have a throttled_update method to control when heavy computation will be executed ( potentially not every update)
* displaying name of ROS node in log when starting up.
* mentioning dropping actions support in changelog.
* Overhauled documentation.
* cosmetics.
* exposing pyros service exceptions for import.
* adding node with mute publisher for tests.
* fixing basic test nodes return message type.
  cosmetics.
* reviewing README.
* changelog for 0.1.0. cosmetics.
* Merge pull request `#43 <https://github.com/asmodehn/pyros/issues/43>`_ from asmodehn/autofix/wrapped2_to3_fix
  Fix "Prefer `format()` over string interpolation operator" issue
* fixing badges after rename.
* Merge pull request `#42 <https://github.com/asmodehn/pyros/issues/42>`_ from asmodehn/autofix/wrapped2_to3_fix
  Fix "Avoid mutable default arguments" issue
* Migrated `%` string formating
* Avoid mutable default arguments
* Merge pull request `#41 <https://github.com/asmodehn/pyros/issues/41>`_ from asmodehn/mp_exception
  Multiprocess
* made namedtuple fields optional like for protobuf protocol.
* fixing zmp tests with namedtuple protocol
* fixing catkin cmakelists after test rename
* Making client exceptions also PyrosExceptions.
* begining of implementation of slowservice node for test. not included in tests yet.
* removed useless hack in travis cmds, fixed typo
* trying quick hack to fix travis build.
* adding status message when creating linksto access catkin generated python modules.
* adding zmp tests to catkin cmakelists.
* added dummy file to fix catkin install.
* small install and deps fixes.
* simplifying traceback response code in node.
* fixing unusable traceback usecase in zmp.
* cosmetics. adding basemsg unused yet.
* moving exception to base package, as they should be usable by the client of this package.
* making pyros exceptions pickleable.
  minor fixes to ensure exception propagation.
* comments
* ros_setup now use of install workspace optional. fixes problems running nodes ( which needs message types ) from nosetests.
* added cleanup methods for transients. it comes in handy sometime ( for ROS topics for example ).
* pretty print dynamic reconfigure request.
* cleanup debug logging.
* adding logic on name was not a good idea. breaks underlying systems relaying on node name like params for ROS.
* removing name from argv, catching keyboard interrupt from pyros ros node.
  cosmetics.
* increasing default timeouts for listing services call form pyros client.
* fixed multiprocess mutli pyros conflict issues with topics with well known rosparam.
  now enforcing first part of node name.
  cosmetics.
* removed useless logging.
* adding basetopic and fixed topic detection in rosinterface.
  zmp service now excepting on timeout.
* fixed exceptions handling and transfer.
  fixed serialization of services and topic classes for ROSinterface.
* now reraise when transient type resolving or transient instance building fails.
  added reinit methods to list of node service to be able to change configuration without restarting the node ( usecase : dynamic reconfigure )
  added option to PyrosROS node to start without dynamic reconfigure (useful for tests and explicit reinit)
  added some PyrosROS tests to check dynamic exposing of topics.
  cleaned up old rostful definitions.
  cosmetics
* cleaning up old action-related code. fixed mores tests.
* fixing how to get topics and services list. commented some useless services ( interactions, ationcs, etc. ).
* changing version number to 0.1.0. preparing for minor release
* refactoring ros emulated setup
* improving and fixing rosinterface tests. still too many failures with rostest.
* fixing tests for Pyros client, and fixed Pyros client discovery logic. cosmetics.
* making RosInterface a child of BaseInterface and getting all Topic and test services to pass. cosmetics.
* improved test structure for rostest and nose to collaborate...
* WIP. reorganising tests, moved inside package, nose import makes it easy. still having problems with rostest.
* fixing testTopic for rostest and nose.
  cosmetics.
* finishing python package rename
* separated rospy / py trick from test.
* fixing testRosInterface rostest to be runnable from python directly, and debuggable in IDE, by emulating ROS setup in testfile.
* implemented functional API, abstract base interface class, mockinterface tests.
* moving and fixing tests.
* Merge branch 'indigo-devel' of https://github.com/asmodehn/pyros into mp_exception
  Conflicts:
  setup.py
  src/rostful_node/rostful_node_process.py
* implemented transferring exception information via protobuf msg.
  readding tblib as dependency required for trusty.
* WIP. starting to change message to be able to just not send the traceback if tblib not found.
* restructuring code and fixing all tests to run with new zmp-based implementation
* now able to use bound methods as services
* changing ros package name after repository rename
* adding python-tblib as catkin dependency
* useful todo comments.
* now using pickle is enough for serialization.
  getting rid of extra dill and funcsig dependencies
* not transmitting function signature anymore. not needed for python style function matching.
* added cloudpickle in possible serializer comments.
* now forwarding all exceptions in service call on node
  fixed all zmp tests.
* fixing all zmp tests since we changed request into args and kwargs
* starting to use dill for serializing functions and params
* fixing setup.py for recent catkin
* protecting rospy from unicode args list
* adding comments with more serialization lib candidates...
* WIP. looking for a way to enforce arguments type when calling a service, and parsing properly when returning an error upon exception.
* getting message to work for both protobuf and pickle. Now we need to choose between tblib and dill for exception serialization.
* adding dill as dependency
* multiprocess simple framework as separate zmp package.
* comments
* transferring exceptions between processes
* fixing all service tests and deadlock gone.
* improved service and node tests. still deadlock sometimes...
* multiprocess service testing okay for discover.
* WIP. starting to use zmq for messaging. simpler than other alternatives.
* WIP implementing service.
* WIP adding mockframework a multiprocess communication framework
* adding mockparam
* adding code health badge
* adding requirements badge
* adding code quality badge
* adding echo tests for mocktopic and mockservice
* renaming populate / extract commands
* Setting up custom message type and tests for mock interface.
* fixing mockmessage and test
* improving mockmessage and tests
* started to build a mock interface, using python types as messages.
  This should help more accurate testing with mock.
* adding six submodule. tblib might need it. otherwise it might come in useful anyway.
* adding tblib to be able to transfer exception between processes.
* fixing travis badge
* adding travis badge
* Merge branch 'indigo-devel' of https://github.com/asmodehn/rostful-node into indigo-devel
* adding rostopic as a test_depend
* Merge pull request `#33 <https://github.com/asmodehn/pyros/issues/33>`_ from asmodehn/travis
  starting travis integration for autotest
* starting travis integration for autotest
* Merge pull request `#32 <https://github.com/asmodehn/pyros/issues/32>`_ from asmodehn/params
  Params
* fixes to make this node work again with rostful
  cosmetics and cleanups
* First implementation to expose params to python the same way as we do for topics and services
* Contributors: AlexV, Cody, alexv

0.0.7 (2015-10-12)
------------------
* Now launching node as subprocess to isolate rospy concurrent behavior and other multiprocess libraries.
* Added basic unit tests
* Refactored all components
