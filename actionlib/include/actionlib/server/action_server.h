/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACTION_LIB_ACTION_SERVER
#define ACTION_LIB_ACTION_SERVER

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/enclosure_deleter.h>
#include <actionlib/goal_id_generator.h>
#include <actionlib/action_definition.h>
#include <actionlib/server/status_tracker.h>
#include <actionlib/server/handle_tracker_deleter.h>
#include <actionlib/server/server_goal_handle.h>

#include <list>

namespace actionlib {
  /**
   * @class ActionServer
   * @brief The ActionServer is a helpful tool for managing goal requests to a
   * node. It allows the user to specify callbacks that are invoked when goal
   * or cancel requests come over the wire, and passes back GoalHandles that
   * can be used to track the state of a given goal request. The ActionServer
   * makes no assumptions about the policy used to service these goals, and
   * sends status for each goal over the wire until the last GoalHandle
   * associated with a goal request is destroyed.
   */
  template <class ActionSpec>
  class ActionServer {
    public:
      //for convenience when referring to ServerGoalHandles
      typedef ServerGoalHandle<ActionSpec> GoalHandle;

    private:
      //generates typedefs that we'll use to make our lives easier
      ACTION_DEFINITION(ActionSpec);

    public:
      /**
       * @brief  Constructor for an ActionServer
       * @param  n A NodeHandle to create a namespace under
       * @param  name The name of the action
       * @param  goal_cb A goal callback to be called when the ActionServer receives a new goal over the wire
       * @param  cancel_cb A cancel callback to be called when the ActionServer receives a new cancel request over the wire
       * @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up
       */
      ActionServer(ros::NodeHandle n, std::string name,
          boost::function<void (GoalHandle)> goal_cb = boost::function<void (GoalHandle)>(),
          boost::function<void (GoalHandle)> cancel_cb = boost::function<void (GoalHandle)>(),
          bool auto_start = true);

      /**
       * @brief  Constructor for an ActionServer
       * @param  n A NodeHandle to create a namespace under
       * @param  name The name of the action
       * @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up
       */
      ActionServer(ros::NodeHandle n, std::string name,
          bool auto_start = true);

      /**
       * @brief  Register a callback to be invoked when a new goal is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerGoalCallback(boost::function<void (GoalHandle)> cb);

      /**
       * @brief  Register a callback to be invoked when a new cancel is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerCancelCallback(boost::function<void (GoalHandle)> cb);

      /**
       * @brief  Explicitly start the action server, used it auto_start is set to false
       */
      void start();

    private:
      /**
       * @brief  Initialize all ROS connections and setup timers
       */
      void initialize();

      /**
       * @brief  Publishes a result for a given goal
       * @param status The status of the goal with which the result is associated
       * @param result The result to publish
       */
      void publishResult(const actionlib_msgs::GoalStatus& status, const Result& result);

      /**
       * @brief  Publishes feedback for a given goal
       * @param status The status of the goal with which the feedback is associated
       * @param feedback The feedback to publish
       */
      void publishFeedback(const actionlib_msgs::GoalStatus& status, const Feedback& feedback);

      /**
       * @brief  The ROS callback for cancel requests coming into the ActionServer
       */
      void cancelCallback(const boost::shared_ptr<const actionlib_msgs::GoalID>& goal_id);

      /**
       * @brief  The ROS callback for goals coming into the ActionServer
       */
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal);

      /**
       * @brief  Publish status for all goals on a timer event
       */
      void publishStatus(const ros::TimerEvent& e);

      /**
       * @brief  Explicitly publish status
       */
      void publishStatus();

      ros::NodeHandle node_;

      ros::Subscriber goal_sub_, cancel_sub_;
      ros::Publisher status_pub_, result_pub_, feedback_pub_;

      boost::recursive_mutex lock_;

      ros::Timer status_timer_;

      std::list<StatusTracker<ActionSpec> > status_list_;

      boost::function<void (GoalHandle)> goal_callback_;
      boost::function<void (GoalHandle)> cancel_callback_;

      ros::Time last_cancel_;
      ros::Duration status_list_timeout_;

      //we need to allow access to our private fields to our helper classes
      friend class ServerGoalHandle<ActionSpec>;
      friend class HandleTrackerDeleter<ActionSpec>;

      GoalIDGenerator id_generator_;
      bool started_;
  };
};

//include the implementation
#include <actionlib/server/action_server_imp.h>
#endif
