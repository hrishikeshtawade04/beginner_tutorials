[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
<h1 align=center> ENPM808X ROS Tutorials </h1>
</p>
<p align="center">
<img src="/readme_images/ROS.png">
</p>
</p>
<p align="center">
Reference for image: <a href='https://www.google.com/imgres?imgurl=https%3A%2F%2Fmoveit.ros.org%2Fassets%2Fimages%2Flogo%2FROS_logo.png&imgrefurl=https%3A%2F%2Fmoveit.ros.org%2F&docid=CyLySsR7n4CjkM&tbnid=yjk2FyriYEe3iM%3A&vet=10ahUKEwjLorK96a7eAhUBjVkKHcARAu0QMwhnKA0wDQ..i&w=680&h=365&bih=672&biw=1301&q=ROS%20image&ved=0ahUKEwjLorK96a7eAhUBjVkKHcARAu0QMwhnKA0wDQ&iact=mrc&uact=8'>link</a>
</p>

## Project Overview
The project covers creating publisher and subscriber in ROS Kinetic. It has two following nodes:
* Talker : Publishes "Happy Halloween" to topic named 'chatter'.
* Listener : Subscibes to the topic named 'chatter' and prints the received messages on the terminal.

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* Ubuntu 16.04

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

## Build Instructions
```
source /opt/ros/kinetic/setup.bash
sudo rm -R catkin_ws (skip if there is not catkin_ws)
mkdir -p catkin_ws/src
cd catkin_ws/
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
source devel/setup.bash
rosrun beginner_tutorials talker
```
To run listener node, open a new terminal window and run following command
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials listener
```
## RQT Graph
Following is the detailed RQT graph upon successful execution of the nodes.
</p>
<p align="center">
<img src="/readme_images/rosgraph.png">
</p>
</p>
