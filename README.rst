.. image:: https://travis-ci.org/asmodehn/rostful-node.svg?branch=indigo-devel
    :target: https://travis-ci.org/asmodehn/rostful-node

.. image:: https://requires.io/github/asmodehn/rostful-node/requirements.svg?branch=mp_exception
     :target: https://requires.io/github/asmodehn/rostful-node/requirements/?branch=mp_exception
     :alt: Requirements Status

.. image:: https://www.quantifiedcode.com/api/v1/project/9a82d3edf0f04500915b0e6c5d3b8751/badge.svg
  :target: https://www.quantifiedcode.com/app/project/9a82d3edf0f04500915b0e6c5d3b8751
  :alt: Code issues

rostful-node
------------

The node embedded in a ROS system to allow rostful to do its job.

This is not meant to be launched by itself.
Instead it should be used as a python package, creating a node for interfacing to the world outside ROS.

Meaning each package depending on rostful-node will create their own ROS node.
It is intended so that the configuration ( what is exposed or not ) can be different for each one of them.
Be careful however in settings where we have multiple similar clients ( like web server scaling ), only one node is needed here.

This node should be launched with roslaunch and follow ros de facto standards, in order to perform tests on it.

