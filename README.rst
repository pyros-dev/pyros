Pyros
=====

.. image:: https://badges.gitter.im/asmodehn/pyros.svg
   :alt: Join the chat at https://gitter.im/asmodehn/pyros
   :target: https://gitter.im/asmodehn/pyros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge

.. image:: https://travis-ci.org/asmodehn/pyros.svg?branch=indigo-devel
    :target: https://travis-ci.org/asmodehn/pyros

.. image:: https://requires.io/github/asmodehn/pyros/requirements.svg?branch=indigo-devel
     :target: https://requires.io/github/asmodehn/pyros/requirements/?branch=indigo-devel
     :alt: Requirements Status

.. image:: https://landscape.io/github/asmodehn/pyros/indigo-devel/landscape.svg?style=flat
   :target: https://landscape.io/github/asmodehn/pyros/indigo-devel
   :alt: Code Health

.. image:: https://www.quantifiedcode.com/api/v1/project/68d207b248dd4b3f89cf48e5de89c461/badge.svg
  :target: https://www.quantifiedcode.com/app/project/68d207b248dd4b3f89cf48e5de89c461
  :alt: Code issues


- If you are a `ROS`_ developer and want to use python inside `ROS`_, you can stop reading and go have a look there instead https://github.com/ros/ros_comm

- If you are a Python developer and are curious about how you can plug a robot in your existing System using python, then keep on reading.

Pyros is a multiprocess interface python package.

It is not meant to run by itself, instead it should be used as a client library on one side,
connecting to a specific process (`ROS node`_) it spawns inside your multiprocess system (`ROS`_) on the other side, in order to interface with it.

`ROS`_ is the main application of pyros, however its abstract base class design aim at developing complete abstraction of such distributed systems and interface with other kinds of multiprocess systems.

This package ultimately doesn't need `ROS`_ to be able to run and perform basic selftests on it.
When using the ros interface however, your `ROS`_ configuration will be dynamically detected by `pyros-setup`_ and setup under the hood.

Do not hesitate to get in touch by leaving an issue, if you think Pyros can help interfacing the multiprocess system you need to communicate with.

ROS usage
---------
Pyros is a blanket under which all the ROS tricks can remain hidden, and not break your usual python workflows.

ROS needs to be installed on your machine for the rosinterface to work.
This package includes :

- a rosnode that will be launched in a separate process to maintain isolation.
- a client library to connect to this node and interface with the `ROS`_ system.

Standard python packages can then use pyros to access the `ROS`_ system via pyros pure python API.
That means each package depending on pyros will create their own ROS node.
It is intended so that the configuration ( what is exposed or not ) can be different for each one of them.
Be careful in settings where we have multiple similar clients ( like web server scaling up, celery multiprocess worker, etc. ), only one pyros node is needed per client type...

Why Pyros ?
^^^^^^^^^^^
Historically Pyros spawned from `rostful`_.
With multiple multiprocess systems like `celery`_ and `flask`_ needing to communicate with ROS underneath, it became necessary to implement a generic python interface that could be used from any usual python software, the way a python developer would expect it.

`rospy`_ is an interface to python inside ROS, but it also enforces some behaviors that are specific to ROS :

- expect the whole ROS environment is already setup before start (``source workspace/devel/setup.bash``)
- initializing a node is mandatory to communicate on topics (but not services)
- only one node can be initialized once per process 
- some global side effects can break things around.

These are not a problem to use python as a language inside a `ROS`_ environment, always using `rospy`_. But they make interfacing a `ROS`_ system with existing python packages much more tricky than needed, and often the outcome is not quite the expected one :

- Most web servers spawn multiple processes to handle concurrent requests, and an interpreter is loaded in each process, which also has the WSGI application executed for each request. That means, if you use `rospy`_ directly you end up with as many nodes as there is web processes (at best). Exactly duplicated nodes which you don't need in your `ROS`_ environment. Also these can take time to start, become inconsistent after a while, etc. This is not what a developer working on a robot wants to happen.
- Custom multiprocess software (like `celery`_) have their own expectation and complex setup regarding global state and how processes are managed by the application, between the import from multiple places, multiple launches, and the way memory is managed. Mixing this with the way `rospy`_ does things is just a recipe for disaster.

Pyros solves these problems by : 

- dynamically loading `ROS`_ environment on python import (with `pyros-setup`_).
- not relying on `ROS`_ to implement multiprocess communications.
- keeping each multiprocess system implementation isolated in their own process.

Supported distributions currently
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ROS : Indigo

This package can also be used from source as a normal ROS package.
Add it to your ROS workspace as a usual repository and everything should work.
If it doesn't work from source as you would expect it, please submit an issue.

Coming up soon
^^^^^^^^^^^^^^

- [ ] a pip package is available to be installed from source (works via rosdep).
- [ ] an ubuntu (virtual env embedded) is provided to install pyros on your Ubuntu Trusty to use pyros from binary.

=> If you want to build a binary package depending on pyros, you should rely on the pip dependency mechanism and embed the appropriate version into your deb (virtual env embedded).

Roadmap
-------

We should do what we can to catch up on IoT platforms design.
Although Pyros focus on pure python design, another package depending on it should enable ROS systems to connect as device to these IoT platforms.

Currently Rostful provide HTTP connection to your robot.
For example another project could provide MQTT connection to your robot, following amazon choice of using MQTT (OASIS Standard).

IoT platforms design :

- Amazon :  http://docs.aws.amazon.com/iot/latest/developerguide/protocols.html

Will NOT do in pyros
--------------------
- Support for Actions (http://wiki.ros.org/actionlib) in pyros itself.

Actions are built upon multiple ROS topics, and can be interfaced like that,
by writing an action client on the other hand ( pure python, javascript on top of rostful, etc. ),
with pyros just forwarding all the required topics.

If this statement happens to be not true, this can be reconsidered.
Additionally when interfacing with other systems "outside" of ROS, services are probably what you want,
maybe also topics sometimes, but probably not actions.
And actions can probably be implemented with just only a couple of services.
The problem here is likely the latency + communication rate that will, anyway, be hard to maintain while moving through layers to go outside of `ROS`_.

- Support for Rocon (http://wiki.ros.org/rocon) in pyros itself.

Although early versions of pyros were supporting rocon features (Rapps, interactions, etc.), the maintenance effort required is too high right now, and anyway it is probably better to have a more solid pyros available to allow people to develop packages on top of pyros to match Rocon features from outside of ROS, if the need arises.

=> We probably will not support anything that is not part of the core of ROS. As long as implementation in another python package is possible using pyros.

.. _ROS : http://wiki.ros.org/
.. _ROS node : http://wiki.ros.org/Nodes
.. _pyros-setup : https://github.com/asmodehn/pyros-setup
.. _rostful : https://github.com/asmodehn/rostful
.. _rospy : https://github.com/ros/ros_comm/tree/indigo-devel/clients/rospy
.. _celery : https://github.com/celery/celery
.. _flask : https://github.com/mitsuhiko/flask
