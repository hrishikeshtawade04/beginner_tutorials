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

