Changelog
=========

0.1.0
-----
* Separated Pyros from Rostful. Pyros as generic multiprocess system interface so Rostful remains pure python web REST API, without multiprocess complexity.
* Separated the pure python multiprocess + communication handling into a small package, waiting for extraction.
* Build an Abstract Base Class structure to make interfacing more testable, and easier to extend.
* Build a mock interface implementation in order to test and verify basic abstract behaviors.
* Moved the rosinterface into it s special place and wrapped it tightly with tests
* added a few methods to do a ros setup dynamically, if it wasn't done before running the package code and if needed

0.0.7
-----
* Now launching node as subprocess to isolate rospy concurrent behavior and other multiprocess libraries.
* Added basic unit tests
* Refactored all components

0.0.2
-----
* Rostful Converted to Catkin
* Migrated from raw python to Flask
* Experimental Rocon support
