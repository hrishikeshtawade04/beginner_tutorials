/**
 *  @file    talker.cpp
 *
 *  @author  Hrishikesh Tawade
 *
 *  @copyright BSD License
 *
 *  @brief ROS publisher node
 *
 *  @section DESCRIPTION
 *
 *  This program publishes the message Happy Halloween
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"


/**
 * @brief   main function
 *
 * @return  0 code functions properly
 */
int main(int argc, char **argv) {
  /// Initializes the talker node
  ros::init(argc, argv, "talker");
  /// Creating a handle to process
  ros::NodeHandle n;
  /// defines the topic and type of message on which the
  /// publisher is going to publish data
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  /// specifying the rate at  which the loop will run
  ros::Rate loop_rate(10);
  /// A count of how many messages we have sent.
  int count = 0;
  while (ros::ok()) {
    /// Creating a message object to stuff data and then publish it.
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Happy Halloween" << count;
    msg.data = ss.str();
    /// prints the sent data on the terminal
    ROS_INFO("%s", msg.data.c_str());
    /// broadcasting the message
    chatter_pub.publish(msg);
    /// necessary only when a subscriber added to the code
    ros::spinOnce();
    /// node sleeps when not publishing to match the publish rate
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

