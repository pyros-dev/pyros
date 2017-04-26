Pyros Talker
============

This example shows how to use pyros with ROS talker example

ROS Talker Launch
-----------------
Run the ROS talker and listener nodes as usual using roslaunch, in a separate terminal : 

```
$ source /opt/ros/indigo/setup.bash 
$ roslaunch roscpp_tutorials talker_listener.launch 
```

Pyros (v0.3.x) Setup
------------------

Pyros makes using a ROS system from a python setup simpler.
From a new terminal, run : 

```
$ mkvirtualenv pyros --system-site-packages
New python executable in /home/alexv/.virtualenvs/pyros/bin/python
Installing setuptools, pip, wheel...done.
```

Checkout this repository, activate hte virtual env 
```
$ git clone https://github.com/asmodehn/pyros-examples.git
$ cd pyros-examples/ROS
```

Activate the virtual environment if it was not already done
```
$ workon pyros
(pyros) $
```

And install the required package (pyros) with the proper interface package for ROS from [PyPI](https://pypi.python.org/pypi)
```
(pyros) $ pip install pyros[ros]
```

The first command to know is pyros_setup. It is installed as a dependency to pyros_interfaces_ros.
It will let you configure how the virtual environment can access the ROS installation on your machine : 
```
(pyros) $ pyros_setup --help
Pyros_setup is a tool to help you manipulate python environment setup.
It is especially useful with ROS and other environments that rely on system python with PYTHONPATH modifications.
Usage: pyros_setup [--pytest | --version | --help]
```

The first time, you should run a self check on pyros_setup.
This will setup the default configuration on your system, by automatically detecting your ROS installation :
```
(pyros) $ pyros_setup --pytest
================================================================= test session starts =================================================================
platform linux2 -- Python 2.7.6, pytest-3.0.7, py-1.4.33, pluggy-0.4.0
rootdir: /home/alexv/Projects/pyros-examples, inifile:
plugins: timeout-1.2.0, hypothesis-3.1.0
collected 1 items 

. INFO pyros_config.confighandler:Loading configuration from /home/alexv/.virtualenvs/pyros/var/pyros_setup-instance/pyros_setup.cfg
WARNING pyros_config.confighandler:Default configuration has been generated in /home/alexv/.virtualenvs/pyros/var/pyros_setup-instance/pyros_setup.cfg
WARNING pyros_setup:Dynamic PyROS setup starting...
WARNING pyros_setup.ros_setup: => Pyros_setup v0.2.1 Emulating ROS setup now for distro indigo and workspaces ()
WARNING pyros_setup.ros_setup:Prepending path /opt/ros/indigo to CMAKE_PREFIX_PATH
WARNING pyros_setup.ros_setup:Prepending path /opt/ros/indigo/share to ROS_PACKAGE_PATH
WARNING pyros_setup.ros_setup:Prepending path /opt/ros/indigo/bin to PATH
WARNING pyros_setup.ros_setup:Prepending path /opt/ros/indigo/lib to LD_LIBRARY_PATH
WARNING pyros_setup.ros_setup:Prepending path /opt/ros/indigo/lib/pkgconfig to PKG_CONFIG_PATH
WARNING pyros_setup.ros_setup:Prepending path /opt/ros/indigo/lib/python2.7/dist-packages to sys.path
Re-adding /opt/ros/indigo/lib/python2.7/dist-packages in front of sys.path
WARNING pyros_setup.ros_setup: => ROS setup emulation done.
WARNING pyros_setup:Dynamic PyROS setup done.
.

============================================================== 1 passed in 0.05 seconds ===============================================================
(pyros) $
```

This will allow you to check in detail your setup and edit the configuration file if necessary.
For more detail about pyros-setup, you should check the [pyros-setup website](https://github.com/asmodehn/pyros-setup.git)


Using Pyros (v0.4.x)
--------------------

If a pyros ROS node was not launched previously on the ROS system, you can dynamically start the pyros node inside the running ROS system, and create a client to connect to it : 
```
(pyros) $ python
Python 2.7.6 (default, Oct 26 2016, 20:30:19) 
[GCC 4.8.4] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> import pyros_interfaces_ros
WARNING:root:ZMQ : Protobuf message implementation not found. Using pickle based protocol
WARNING:pyros_interfaces_ros:loading pyros_setup and configuring your ROS environment
WARNING:pyros_setup:Dynamic PyROS setup starting...
WARNING:pyros_setup.ros_setup: => Pyros_setup v0.2.1 Emulating ROS setup now for distro indigo and workspaces ()
WARNING:pyros_setup.ros_setup:Prepending path /opt/ros/indigo to CMAKE_PREFIX_PATH
WARNING:pyros_setup.ros_setup:Prepending path /opt/ros/indigo/share to ROS_PACKAGE_PATH
WARNING:pyros_setup.ros_setup:Prepending path /opt/ros/indigo/bin to PATH
WARNING:pyros_setup.ros_setup:Prepending path /opt/ros/indigo/lib to LD_LIBRARY_PATH
WARNING:pyros_setup.ros_setup:Prepending path /opt/ros/indigo/lib/pkgconfig to PKG_CONFIG_PATH
WARNING:pyros_setup.ros_setup:Prepending path /opt/ros/indigo/lib/python2.7/dist-packages to sys.path
Re-adding /opt/ros/indigo/lib/python2.7/dist-packages in front of sys.path
WARNING:pyros_setup.ros_setup: => ROS setup emulation done.
WARNING:pyros_setup:Dynamic PyROS setup done.
WARNING:rosout:Failed to load Python extension for LZ4 support. LZ4 compression will not be available.
>>> subproc = pyros_interfaces_ros.PyrosROS("pyrosnode")
>>> client_conn = subproc.start()
[pyrosnode] Node started as [9226 <= ipc:///tmp/zmp-pyrosnode-9wwwUn/services.pipe]
[INFO] [WallTime: 1493190663.803954] RosInterface pyrosnode node started with args : []
[INFO] [WallTime: 1493190663.808503] [pyros_interfaces_ros.ros_interface] ROS Interface initialized with:
        -    services : []
        -    publishers : []
        -    subscribers : []
        -    params : []
        -    enable_cache : False
```
If you connect onto your ROS system, you will see a node has been added : 
```
alexv@AlexV-Linux:~$ source /opt/ros/indigo/setup.bash 
alexv@AlexV-Linux:~$ rosnode list 
/listener
/pyrosnode
/rosout
/talker
```
More details about this on the [pyros-rosinterface website](https://github.com/asmodehn/pyros-rosinterface.git)

We can now, from python, create a client and connect to our new pyros ROS node
```
>>> import pyros 
>>> import pyros.client
>>> client = pyros.client.PyrosClient(client_conn)

```

_**TODO : connect to an existing pyros node on ROS system**_


Now you can connect to the Pyros ROS node to introspect and interract with the ROS system.
```
>>> client = pyros.PyrosClient(client_conn)
>>> client.subscribers()
{}
>>> client.publishers()
{}
>>> client.services()
{}
>>> client.params()
{}
``` 

To finally stop the node you launched, you can : 
```
>>> subproc.shutdown()
Shutdown initiated
```