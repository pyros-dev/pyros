Pyros
=====

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


- If you are a ROS developer and want to use python inside ROS, you can stop reading and go have a look there instead https://github.com/ros/ros_comm

- If you are a Python developer and are curious about how you can plug a robot in your existing System using python, then keep on reading.

Pyros is a multiprocess interface python package.
It is not meant to run by itself, instead it should be used as a client library,
connecting to a specific process inside your multiprocess system in order to interface with it.

ROS usage
---------
ROS needs to be installed on your machine for the rosinterface to work.
This package includes :
- a rosnode that will be launched in a separate process to maintain isolation.
- a client library to connect to this node and interface with the ROS system.
This node doesn't need ROS to run and perform tests on it.

This package can also be added to your ROS workspace as a normal ROS package and everything should work.
Other (python or ROS) packages can use pyros to access the ROS system.
Meaning each package depending on pyros will create their own ROS node.
It is intended so that the configuration ( what is exposed or not ) can be different for each one of them.
Be careful in settings where we have multiple similar clients ( like web server scaling ), only one pyros node is needed.

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
The problem here is likely the latency + communication rate that will, anyway, be hard to maintain while moving through layers to go outside of ROS.

