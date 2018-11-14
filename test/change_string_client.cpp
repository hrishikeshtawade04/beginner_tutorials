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
 *  @file    change_string_client.cpp
 *
 *  @author  Hrishikesh Tawade
 *
 *  @copyright BSD License
 *
 *  @brief ROS testing node
 *
 *  @section DESCRIPTION
 *
 *  This program tests whether the  service changeBasesString
 *  is functioning properly
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/changeBaseString.h>

/**
 * @brief      Tests whether the service exists and changes text
 *
 * @param[in]  ChangeBaseStringTest   Name of test suite
 * @param[in]  checkChangeString  Name of the test
 */
TEST(ChangeBaseStringTest, checkChangeString) {
  /// creating a node handle
  ros::NodeHandle n;
  /// declaring service client
  ros::ServiceClient client = n.serviceClient
      < beginner_tutorials::changeBaseString > ("change_string");
  /// checking if the service exists
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
  /// creatign service object
  beginner_tutorials::changeBaseString srv;
  /// requesting the new string to service
  srv.request.newString = "HalloweenOver";
  client.call(srv);
  /// comparing the service response with the requested string
  EXPECT_STREQ("HalloweenOver", srv.response.changedString.c_str());
}

/**
 * @brief      main
 *
 * @param      argc  The count of the code and arguments passed
 * @param      argv  The pointer to pointers of arguments passed
 *
 * @return     Nothing
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "change_string_client");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


