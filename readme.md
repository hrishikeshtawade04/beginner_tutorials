[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Project Overview
The project covers creating publisher and subscriber in ROS. It has two following nodes:
* Talker
* Listener

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* Ubuntu 16.04

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/hrishikeshtawade04/beginner_tutorials.git
cd ..
catkin_make
```
## Running Instructions
To start roscore, open a new terminal window and run following command
```
source /opt/ros/kinetic/setup.bash
roscore
```
To run talker node, open a new terminal window and run following command
```
cd <path to catkin_ws>
source /devel/setup.bash
rosrun beginner_tutorials talker
```
To run listener node, open a new terminal window and run following command
```
cd <path to catkin_ws>
source /devel/setup.bash
rosrun beginner_tutorials listener
```
