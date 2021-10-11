/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Smart Robotics BV.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//! \author Ramon Wijnands

#include <actionlib/TestAction.h>
#include <actionlib/server/simple_action_server.h>
#include <gtest/gtest.h>
#include <stdlib.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

using namespace actionlib;

TEST(SimpleActionServerConstruction, test_name_cb_autostart) {
  SimpleActionServer<TestAction>::ExecuteCallback callback = [](const TestGoalConstPtr&){};
  SimpleActionServer<TestAction> as("name", callback, false);
}

TEST(SimpleActionServerConstruction, test_name_autostart) {
  SimpleActionServer<TestAction> as("name", false);
}

TEST(SimpleActionServerConstruction, test_name) {
  SimpleActionServer<TestAction> as("name");
}

TEST(SimpleActionServerConstruction, test_name_cb) {
  SimpleActionServer<TestAction>::ExecuteCallback callback = [](const TestGoalConstPtr&){};
  SimpleActionServer<TestAction> as("name", callback);
}

TEST(SimpleActionServerConstruction, test_nh_name_cb_autostart) {
  ros::NodeHandle nh;
  SimpleActionServer<TestAction>::ExecuteCallback callback = [](const TestGoalConstPtr&){};
  SimpleActionServer<TestAction> as(nh, "name", callback, false);
}

TEST(SimpleActionServerConstruction, test_nh_name_autostart) {
  ros::NodeHandle nh;
  SimpleActionServer<TestAction> as(nh, "name", false);
}

TEST(SimpleActionServerConstruction, test_nh_name) {
  ros::NodeHandle nh;
  SimpleActionServer<TestAction> as(nh, "name");
}

TEST(SimpleActionServerConstruction, test_nh_name_cb) {
  ros::NodeHandle nh;
  SimpleActionServer<TestAction>::ExecuteCallback callback = [](const TestGoalConstPtr&){};
  SimpleActionServer<TestAction> as(nh, "name", callback);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "simple_action_server_construction");

  return RUN_ALL_TESTS();
}
