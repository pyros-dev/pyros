.. _index:

Pyros
=====

This `Python package`_ and `ROS`_ package interface multiple multi-process systems.

For example you can have a ROS system on one side, with his different nodes, services, etc.
and a python multiprocess system (celery, tornado, etc.) on the other side, with also his own nodes, services, etc.

Pyros allows them to communicate, while keeping each other isolated in their own process context.

If you want to know more about how Pyros does this, you can have a look at the :ref:`overview`.

.. include:: weblinks.rst


Contents:

.. toctree::
   :maxdepth: 2

   readme_link
   overview
   roadmap
   pyros_internals
   zmp_internals
   changelog_link

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
