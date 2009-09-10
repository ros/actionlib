/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

//! \author Vijay Pradeep

#include <gtest/gtest.h>
#include <actionlib/TestAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace actionlib;

TEST(SimpleClient, easy_tests)
{
  ros::NodeHandle n;
  SimpleActionClient<TestAction> client(n, "reference_action");

  bool started = client.waitForActionServerToStart(ros::Duration(10.0));

  ros::Duration sleep_dur(1.0);
  sleep_dur.sleep();

  ASSERT_TRUE(started);


  TestGoal goal;
  bool finished;

  goal.goal = 1;
  client.sendGoal(goal);
  finished = client.waitForGoalToFinish(ros::Duration(10.0));
  ASSERT_TRUE(finished);
  EXPECT_TRUE( client.getTerminalState() == TerminalState::SUCCEEDED)
      << "Expected [SUCCEEDED], but terminal state is [" << client.getTerminalState().toString() << "]";

  goal.goal = 2;
  client.sendGoal(goal);
  finished = client.waitForGoalToFinish(ros::Duration(10.0));
  ASSERT_TRUE(finished);
  EXPECT_TRUE( client.getTerminalState() == TerminalState::ABORTED)
      << "Expected [ABORTED], but terminal state is [" << client.getTerminalState().toString() << "]";

  goal.goal = 3;
  client.sendGoal(goal);
  finished = client.waitForGoalToFinish(ros::Duration(10.0));
  ASSERT_TRUE(finished);
  EXPECT_TRUE( client.getTerminalState() == TerminalState::REJECTED)
      << "Expected [REJECTED], but terminal state is [" << client.getTerminalState().toString() << "]";

}

void spinThread()
{
  ros::NodeHandle nh;
  ros::spin();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "simple_client_test");

  boost::thread spin_thread(&spinThread);

  int result = RUN_ALL_TESTS();

  ros::shutdown();

  spin_thread.join();

  return result;
}
