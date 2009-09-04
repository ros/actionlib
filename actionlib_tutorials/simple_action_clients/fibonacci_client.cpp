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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *
 *********************************************************************/

/* Author: Melonee Wise */
#include <ros/node.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>



int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci"); 

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true); 

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForActionServerToStart(); //will wait for infinite time
 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action 
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);
  
  //wait for the action to return
  bool finished_before_timeout = ac.waitForGoalToFinish(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::TerminalState state = ac.getTerminalState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else  
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
