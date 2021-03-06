/**
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

