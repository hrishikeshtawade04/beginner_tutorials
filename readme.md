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
git clone -b Week10_HW https://github.com/hrishikeshtawade04/beginner_tutorials.git
cd ..
catkin_make
```
## Running Instructions
To start roscore, open a new terminal window and run following command
```
source /opt/ros/kinetic/setup.bash
roscore
```
To run talker node, open a new terminal window and run following command. The frequncy is the output frequency of talker messages on the terminal. Frequency values should be positve and more then zero othewise it throws an error message.
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials talker <frequency>
```
To run listener node, open a new terminal window and run following command
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials listener
```
## Running ROS launch demo
To run the launch file so that both talker and listener node opens together run the following command. The frequency is the output frequncy of talker messages on the terminal. Frequncy values should be positve and more then zero othewise it throws an error message. If the frequency is not mentioned it will run the default frequency of 10.
```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials talkAndListen.launch freq:=<frequency>
```
## Running Service
The service change_string changes the outputted based string by talker and the changes gets reflected both in talker and listener. Following are the instructions to run the service assuming that  talker and listener nodes are running using any  of the above methods.
```
cd <path to catkin_ws>
source devel/setup.bash
rosservice call /change_string "HalloweenOver"
```
## RQT Graph
Following is the detailed RQT graph upon successful execution of the nodes.
</p>
<p align="center">
<img src="/readme_images/rosgraph.png">
</p>
</p>
