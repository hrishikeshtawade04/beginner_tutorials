/**
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, Hrishikesh Tawade
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  @copyright (c) BSD
 *
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
#include "beginner_tutorials/changeBaseString.h"

/// Initialize the base string to print
extern std::string msgToBeSent = "Happy Halloween";

/**
 * @brief chnages the base string of talker
 * @param req new string from service client
 * @param res response of the service
 * @return true when service function executes properly
 */
bool change(const beginner_tutorials::changeBaseString::Request &req,
            const beginner_tutorials::changeBaseString::Response &res) {
  /// assigns requested string to response string
  res.changedString = req.newString;
  msgToBeSent = res.changedString;
  return true;
}

/**
 * @brief   main function
 *
 * @return  0 code functions properly
 */
int main(int argc, char **argv) {
  /// Initializes the talker node
  ros::init(argc, argv, "talker");
  /// Initializing frequency
  int freq = 0;
  /// Checking if we have two arguments
  if (argc == 2) {
    /// recording the argument passed
    ROS_DEBUG_STREAM("Argument is " << argv[1]);
    /// converts string to integer
    freq = atoi(argv[1]);
    if (freq == 0) {
      ROS_ERROR_STREAM(
          "This argument not valid. Frequency set to default value 10");
      freq = 10;
    }

  } else {
    /// shuts down the node when no argument given
    ROS_FATAL_STREAM("No frequency argument was passed.");
    ros::shutdown();
    return 1;
  }
  /// Creating a handle to process
  ros::NodeHandle n;
  /// defines the topic and type of message on which the
  /// publisher is going to publish data
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  /// defining service server
  ros::ServiceServer server = n.advertiseService("change_string", change);
  /// specifying the rate at  which the loop will run
  ros::Rate loop_rate(freq);
  /// A count of how many messages we have sent.
  int count = 0;
  while (ros::ok()) {
    /// Creating a message object to stuff data and then publish it.
    std_msgs::String msg;
    std::stringstream ss;
    ss << msgToBeSent << count;
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
    ROS_DEBUG_STREAM("Argument is " << count);
  }

  return 0;
}


