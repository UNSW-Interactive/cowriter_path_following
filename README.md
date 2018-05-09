# cowriter_path_following
===========================

This package contains a set of nodes that are used to manage the tablet interface for the path following activity, as well as nodes to control the robot behavior.

Tested with ROS Kinetic on Ubuntu 16.04 (LTS)

Install & Compile:
---------------
```
$ cd ~/catkin_ws/src/
$ git clone  https://github.com/chili-epfl/cowriter_path_following.git
$ cd ../
$ catkin_make
```

Running the tablet app nodes:
---------------
Run `cowriter_path_following/nodes/TargetActivity.py`to launch the writing app. 
The main node that opens two windows, one for writing the other one to manage the parameters


Launch the robot control nodes
-----------------------------

```
$ roslaunch cowriter_path_following launch.launch nao_port:=... nao_ip:=... [...other options]

```
