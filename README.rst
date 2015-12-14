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

pyros
-----

The node embedded in a ROS system to allow rostful, celery, and other multi process python libraries to work with ROS

This is not meant to be launched by itself.
Instead it should be used as a python package, creating a node for interfacing to the world outside ROS.

Meaning each package depending on pyros will create their own ROS node.
It is intended so that the configuration ( what is exposed or not ) can be different for each one of them.
Be careful however in settings where we have multiple similar clients ( like web server scaling ), only one node is needed here.

This node should be launched with roslaunch and follow ros de facto standards, in order to perform tests on it.

Will NOT do in pyros
--------------------
- Support for Actions (http://wiki.ros.org/actionlib) in pyros itself.
Actions are built upon multiple ROS topics, and can be interfaced like that,
by writing an action client on the other hand ( pure python, javascript on top of rostful, etc. ),
with pyros just forwarding all the required topics. If this statement happens to be not true, this can be reconsidered.
Additionally when interfacing with other systems "outside" of ROS, services are probably what you want,
maybe also topics, but probably not actions.
And actions can probably be implemented with just only a couple services.
The problem here is likely the latency + communication rate that will, anyway, be hard to maintain while moving through layers to go outside of ROS.

