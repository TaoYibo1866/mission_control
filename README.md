# ActionLite

## 介绍
轻量化ros::action，并基于此实现一个状态机例程
## Build
```Bash
$ cd ~/catkin_ws/src
$ git clone https://gitee.com/harbin-institute-of-technology-csc/mission_control.git
$ cd ~/catkin_ws
$ catkin_make
```
## Run
```Bash
$ roslaunch mission_control demo.launch
$ rosservice call /keyboard_control/preempt "{}"
```