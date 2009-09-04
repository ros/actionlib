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
 **********************************************************************/

/* Author: Melonee Wise */
#include <sstream>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
public:
    
  FibonacciAction(std::string name) : 
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1)),
    action_name_(name)
  {

    std::stringstream ss;
    // check if the configuration is loaded on the param server
    if(!nh_.getParam(action_name_ + "/seed0", seed0_))
    {   
      // throw an exception if the param is not there and exit the action server
      ss << action_name_.c_str() << ": Aborted, seed0 param was not set.";
      throw ss.str();
    }

    if(!nh_.getParam(action_name_ + "/seed1", seed1_))
    {
      ss << action_name_.c_str() << ": Aborted, seed1 param was not set.";
      throw ss.str();
    }
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
   
    // helper variables
    ros::Rate r(1); 
    std::vector<int> sequence;
    int temp;  
    bool success = true;
    
    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(seed0_);
    feedback_.sequence.push_back(seed1_);

    // publish some info to the console for the user 
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
        
    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {        
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback 
      as_.publishFeedback(feedback_);
      // this sleep is not necessary
      r.sleep(); 
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;
  int seed0_, seed1_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  try
  {
    FibonacciAction fibonacci(ros::this_node::getName());
    ros::spin();
  }
  catch(std::string str) 
  {
    ROS_ERROR(str.c_str());
  }
  return 0;
}
