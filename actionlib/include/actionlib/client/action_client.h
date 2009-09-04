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

#ifndef ACTIONLIB_ACTION_CLIENT_H_
#define ACTIONLIB_ACTION_CLIENT_H_

#include <boost/thread/condition.hpp>

#include "ros/ros.h"
#include "ros/callback_queue_interface.h"
#include "actionlib/client/client_helpers.h"

namespace actionlib
{

/**
 * \brief Full interface to an ActionServer
 *
 * ActionClient provides a complete client side implementation of the ActionInterface protocol.
 * It provides callbacks for every client side transition, giving the user full observation into
 * the client side state machine.
 */
template <class ActionSpec>
class ActionClient
{
public:
  typedef ClientGoalHandle<ActionSpec> GoalHandle;

private:
  ACTION_DEFINITION(ActionSpec);
  typedef ActionClient<ActionSpec> ActionClientT;
  typedef boost::function<void (GoalHandle) > TransitionCallback;
  typedef boost::function<void (GoalHandle, const FeedbackConstPtr&) > FeedbackCallback;
  typedef boost::function<void (const ActionGoalConstPtr)> SendGoalFunc;

public:
  /**
   * \brief Simple constructor
   *
   * Constructs an ActionClient and sets up the necessary ros topics for the ActionInterface
   * \param name The action name. Defines the namespace in which the action communicates
   * \param queue CallbackQueue from which this action will process messages.
   *              The default (NULL) is to use the global queue
   */
  ActionClient(const std::string& name, ros::CallbackQueueInterface* queue = NULL) : n_(name)
  {
    initClient(queue);
  }

  /**
   * \brief Constructor with namespacing options
   *
   * Constructs an ActionClient and sets up the necessary ros topics for the ActionInterface,
   * and namespaces them according the a specified NodeHandle
   * \param n The node handle on top of which we want to namespace our action
   * \param name The action name. Defines the namespace in which the action communicates
   * \param queue CallbackQueue from which this action will process messages.
   *              The default (NULL) is to use the global queue
   */
  ActionClient(const ros::NodeHandle& n, const std::string& name, ros::CallbackQueueInterface* queue = NULL) : n_(n, name)
  {
    initClient(queue);
  }

  /**
   * \brief Sends a goal to the ActionServer, and also registers callbacks
   * \param transition_cb Callback that gets called on every client state transition
   * \param feedback_cb Callback that gets called whenever feedback for this goal is received
   */
  GoalHandle sendGoal(const Goal& goal,
                      TransitionCallback transition_cb = TransitionCallback(),
                      FeedbackCallback   feedback_cb   = FeedbackCallback())
  {
    ROS_DEBUG("about to start initGoal()");
    GoalHandle gh = manager_.initGoal(goal, transition_cb, feedback_cb);
    ROS_DEBUG("Done with initGoal()");

    return gh;
  }

  /**
   * \brief Cancel all goals currently running on the action server
   *
   * This preempts all goals running on the action server at the point that
   * this message is serviced by the ActionServer.
   */
  void cancelAllGoals()
  {
    actionlib_msgs::GoalID cancel_msg;
    // CancelAll policy encoded by stamp=0, id=0
    cancel_msg.stamp = ros::Time(0,0);
    cancel_msg.id = ros::Time(0,0);
    cancel_pub_.publish(cancel_msg);
  }

  /**
   * \brief Cancel all goals that were stamped at and before the specified time
   * \param time All goals stamped at or before `time` will be canceled
   */
  void cancelGoalsAtAndBeforeTime(const ros::Time& time)
  {
    ActionGoal cancel_msg;
    cancel_msg.goal_id.stamp = time;
    cancel_msg.goal_id.id = ros::Time(0,0);
    goal_pub_.publish(cancel_msg);
  }

  /**
   * \brief Waits for the ActionServer to connect to this client
   * Often, it can take a second for the action server & client to negotiate
   * a connection, thus, risking the first few goals to be dropped. This call lets
   * the user wait until the network connection to the server is negotiated
   */
  bool waitForActionServerToStart(const ros::Duration& timeout = ros::Duration(0,0) )
  {
    if (timeout < ros::Duration(0,0))
      ROS_WARN("Timeouts can't be negative. Timeout is [%.2fs]", timeout.toSec());

    ros::Time timeout_time = ros::Time::now() + timeout;

    boost::mutex::scoped_lock lock(server_connection_mutex_);

    if (server_connected_)
      return true;

    // Hardcode how often we check for node.ok()
    ros::Duration loop_period = ros::Duration().fromSec(.1);

    while (n_.ok() && !server_connected_)
    {
      // Determine how long we should wait
      ros::Duration time_left = timeout_time - ros::Time::now();

      // Check if we're past the timeout time
      if (timeout != ros::Duration(0,0) && time_left <= ros::Duration(0,0) )
        break;

      // Truncate the time left
      if (time_left > loop_period)
        time_left = loop_period;

      server_connection_condition_.timed_wait(lock, boost::posix_time::milliseconds(time_left.toSec() * 1000.0f));
    }

    return server_connected_;
  }

private:
  ros::NodeHandle n_;

  ros::Subscriber feedback_sub_;
  ros::Publisher  goal_pub_;
  ros::Publisher  cancel_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  GoalManager<ActionSpec> manager_;

  boost::mutex server_connection_mutex_;
  boost::condition server_connection_condition_;
  bool server_connected_;

  void sendGoalFunc(const ActionGoalConstPtr& action_goal)
  {
    goal_pub_.publish(action_goal);
  }

  void sendCancelFunc(const actionlib_msgs::GoalID& cancel_msg)
  {
    cancel_pub_.publish(cancel_msg);
  }

  void initClient(ros::CallbackQueueInterface* queue)
  {
    // Start publishers and subscribers
    server_connected_ = false;
    goal_pub_ = queue_advertise<ActionGoal>("goal", 1, boost::bind(&ActionClient::serverConnectionCb, this, _1), queue);
    cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("cancel", 1, true);
    manager_.registerSendGoalFunc(boost::bind(&ActionClientT::sendGoalFunc, this, _1));
    manager_.registerCancelFunc(boost::bind(&ActionClientT::sendCancelFunc, this, _1));

    status_sub_   = queue_subscribe("status",   1, &ActionClientT::statusCb,   this, queue);
    feedback_sub_ = queue_subscribe("feedback", 1, &ActionClientT::feedbackCb, this, queue);
    result_sub_   = queue_subscribe("result",   1, &ActionClientT::resultCb,   this, queue);
  }

  template <class M>
  ros::Publisher queue_advertise(const std::string& topic, uint32_t queue_size,
                                 const ros::SubscriberStatusCallback& connect_cb,
                                 ros::CallbackQueueInterface* queue)
  {
    ros::AdvertiseOptions ops;
    ops.init<M>(topic, queue_size, connect_cb, NULL);
    ops.tracked_object = ros::VoidPtr();
    ops.latch = false;
    ops.callback_queue = queue;
    return n_.advertise(ops);
  }

  template<class M, class T>
  ros::Subscriber queue_subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, ros::CallbackQueueInterface* queue)
  {
    ros::SubscribeOptions ops;
    ops.init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = ros::TransportHints();
    ops.callback_queue = queue;
    return n_.subscribe(ops);
  }

  void statusCb(const actionlib_msgs::GoalStatusArrayConstPtr& status_array)
  {
    manager_.updateStatuses(status_array);
  }

  void feedbackCb(const ActionFeedbackConstPtr& action_feedback)
  {
    manager_.updateFeedbacks(action_feedback);
  }

  void resultCb(const ActionResultConstPtr& action_result)
  {
    manager_.updateResults(action_result);
  }

  void serverConnectionCb(const ros::SingleSubscriberPublisher& pub)
  {
    boost::mutex::scoped_lock lock(server_connection_mutex_);
    server_connected_ = true;
    server_connection_condition_.notify_all();
  }
};


}

#endif
