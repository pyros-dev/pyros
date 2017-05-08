# pyros-examples
A Set of Examples for Pyros

Assumptions
--------------

- OS is Ubuntu Trusty
- ROS is already known. If not, get informations and tutorials on http://ros.org

ROS Indigo System Setup
-----------------------
- Install useful python packages if you don't already have them :
```bash
$ sudo apt-get install virtualenvwrapper
```
- Install ROS Indigo : http://wiki.ros.org/indigo/Installation/Ubuntu
TL;DR : 
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop
$ sudo rosdep init
```

Examples
--------

These examples will show you how to :
- launch an existing ROS system (using ROS packages)
- install & configure pyros in a virtual environment
- use pyros to interract dynamically with the running ros system.

Notes
-----

Pyros is usable as a usual python package, and can be used from any python environment.

Pyros is also extensible to support other multiprocessing environments, but ROS is the first complete implementation available.

