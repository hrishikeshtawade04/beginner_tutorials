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
git clone -b Week11_HW https://github.com/hrishikeshtawade04/beginner_tutorials.git
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
roslaunch beginner_tutorials talkAndListen.launch freq:=<frequency> enableBag:=false
```
## Running Service
The service change_string changes the outputted based string by talker and the changes gets reflected both in talker and listener. Following are the instructions to run the service assuming that  talker and listener nodes are running using any  of the above methods.
```
cd <path to catkin_ws>
source devel/setup.bash
rosservice call /change_string "HalloweenOver"
```
## Inspecting TF Frames
The talker file publishes the transform of talk frame with respect to world frame. You can use tf_echo and rqt_tf_tree to check whether the frames are broadcasting properly. Considering above launch file is still running run below commands to inspect the tf frames.
 - tf_echo command below outputs the transform from /talk frame with respect to the /world frame.

```
source /opt/ros/kinetic/setup.bash
rosrun tf tf_echo /world /talk
```
It will give the following output.
```
At time 1542156563.691
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.000, 0.000, 0.959, -0.284]
            in RPY (radian) [0.000, -0.000, -2.566]
            in RPY (degree) [0.000, -0.000, -147.042]
At time 1542156564.391
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.000, 0.000, 0.959, -0.284]
            in RPY (radian) [0.000, -0.000, -2.566]
            in RPY (degree) [0.000, -0.000, -147.042]
At time 1542156565.391
```
-To visualize the frames open a new window and run following command.
```
rosrun tf view_frames
evince frames.pdf
```
-To visualize using rqt_tf_tree run following command in new terminal and choose tf_tree from Plugins tab
```
rosrun rqt_tf_tree rqt_tf_tree
```
### Running rostest
The tests can be found in the test folder and it also has a launch file to run the talker node and its tests together.
```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
cd <path to catkin_ws>
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```
or you can also run it using launch file by following running command.
```
rostest beginner_tutorials change_string_client.launch
```
And the output is as follows.
```
... logging to /home/hrishikesh/.ros/log/rostest-hrishikeshtawade-19144.log
[ROSUNIT] Outputting test results to /home/hrishikesh/.ros/test_results/beginner_tutorials/rostest-test_change_string_client.xml
[ INFO] [1542157714.799783117]: Happy Halloween0
[ INFO] [1542157714.899700745]: Happy Halloween1
[ INFO] [1542157714.999678902]: HalloweenOver2
[ INFO] [1542157715.099681026]: HalloweenOver3
[ INFO] [1542157715.199752910]: HalloweenOver4
[Testcase: testchange_string_client] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-change_string_client/checkChangeString][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/hrishikesh/.ros/log/rostest-hrishikeshtawade-19144.log

```
## Running Bag file
The commands to record talker node's data in bag file has already been added in the talkAndListen launch file which can be enabled to record by setting enableBag argument to true as below. The below set of commands record the talker data in the results folder in storedTalker.bag file. To stop the recording press Ctrl-C in the main terminal where you ran the below commands.

```
sudo killall -9 roscore   (to stop any ongoing roscore)
sudo killall -9 rosmaster (to stop any ongoing roscore)
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials talkAndListen.launch freq:=<frequency> enableBag:=true
```
To verify the .bag file run the following command.
```
cd <path to repository>/results
rosbag info storedTalker.bag
```
It should give similar output as following.
```
path:        storedTalker.bag
version:     2.0
duration:    21.4s
start:       Nov 13 2018 20:24:03.00 (1542158643.00)
end:         Nov 13 2018 20:24:24.38 (1542158664.38)
size:        244.7 KB
messages:    1234
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      212 msgs    : std_msgs/String   
             /rosout       407 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   403 msgs    : rosgraph_msgs/Log
             /tf           212 msgs    : tf2_msgs/TFMessage

```
Now to playback the .bag file and check it, first run the below command in one terminal.
```
roscore
```
Then run below in another terminal.
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials listener
```
Now run the following command in new terminal play the storedTalker.bag file and the results should refect in the listener node.
```
cd <path to repository>/results
rosbag play storedTalker.bag
```
To disable the rosbag recording you can use following command.
```
roslaunch beginner_tutorials talkAndListen.launch freq:=<frequency> enableBag:=false
```
## RQT Graph
Following is the detailed RQT graph upon successful execution of the nodes.
</p>
<p align="center">
<img src="/readme_images/rosgraph.png">
</p>
</p>
