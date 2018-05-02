# cowriter_path_following
===========================

Install & Compile:
---------------
```
$ cd ~/catkin_ws/src/
$ git clone  https://github.com/chili-epfl/cowriter_path_following.git
$ cd ../
$ catkin_make
```

Launch and run
-----------------------------


```
$ roslaunch cowriter_path_following launch.launch nao_port:=... nao_ip:=... [...other options]

```
run:
$ python "nodes/TargetActivity.py"
from the cowriter_path_following directory
