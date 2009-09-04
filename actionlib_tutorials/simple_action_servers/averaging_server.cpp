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

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/AveragingAction.h>

class AveragingAction
{
public:
    
  AveragingAction(std::string name) : 
    as_(nh_, name),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&AveragingAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
  }

  ~AveragingAction(void)
  {
  }

  void goalCB()
  {
    // reset helper variables
    data_count_ = 0;
    sum_ = 0;
    sum_sq_ = 0;
    // accept the new goal
    goal_ = (*as_.acceptNewGoal()).samples;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const std_msgs::Float32::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;
    //compute the std_dev and mean of the data 
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    as_.publishFeedback(feedback_);

    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    } 
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::AveragingAction> as_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  actionlib_tutorials::AveragingFeedback feedback_;
  actionlib_tutorials::AveragingResult result_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "averaging");

  AveragingAction averaging(ros::this_node::getName());
  ros::spin();

  return 0;
}
