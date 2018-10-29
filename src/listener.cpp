/**
 *  @file    listener.cpp
 *
 *  @author  Hrishikesh Tawade
 *
 *  @copyright BSD License
 *
 *  @brief ROS listener node
 *
 *  @section DESCRIPTION
 *
 *  This program subscribes to the chatter topic and displays its data
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief Prints data on terminal when data is arrived on topic
 * @param msg Takes input the data published by talker
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  /// printing the data on terminal
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @brief defines the listener node that subscribes to
 *        the chatter topic and prints the data on it
 * @param argc number of arguments given from command line
 * @param argv array of character pointers that points to
 *        passed arguments
 * @return 0 when program runs successfully
 */
int main(int argc, char **argv) {
  /// Initializes the node listener
  ros::init(argc, argv, "listener");
  /// Creating a handle to process
  ros::NodeHandle n;
  /// defines the subscriber subscribing to the chatter topic
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  /// entering a loop while pumping callbacks.
  ros::spin();
  return 0;
}

