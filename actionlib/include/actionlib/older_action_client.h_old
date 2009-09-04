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

#ifndef ACTION_TOOLS_ROBUST_ACTION_CLIENT_H_
#define ACTION_TOOLS_ROBUST_ACTION_CLIENT_H_

#include <boost/thread.hpp>
#include "ros/ros.h"
#include "action_tools/GoalStatus.h"
#include "action_tools/Preempt.h"
#include "action_tools/EnclosureDeleter.h"

#include "action_tools/one_shot_timer.h"

#define setState(next_state) \
{ \
  ROS_DEBUG("Setting ClientState to " #next_state);\
  client_state_ = next_state;\
}

namespace action_tools
{


namespace TerminalStatuses
{
  enum TerminalStatus  { REJECTED, PREEMPTED, SUCCEEDED, ABORTED, TIMED_OUT, IGNORED, LOST } ;
}


template <class ActionGoal, class Goal, class ActionResult, class Result>
class ActionClient
{
public:
  typedef boost::function<void (const TerminalStatuses::TerminalStatus&, const boost::shared_ptr<const Result>&)> CompletionCallback;
  typedef boost::function<void ()> AckTimeoutCallback;
  typedef boost::function<void ()> PreemptTimeoutCallback;
  typedef ActionClient<ActionGoal, Goal, ActionResult, Result> ActionClientT;
  typedef boost::function<void (void)> FilledCompletionCallback;
  typedef boost::shared_ptr<const ActionResult> ActionResultConstPtr;
  typedef boost::shared_ptr<const Result> ResultConstPtr;
  //typedef boost::function<void (const ros::TimerEvent& e)> AckTimeoutCallback;

  ActionClient(std::string name, ros::NodeHandle nh = ros::NodeHandle(),
               bool expecting_result=false,
               const ros::Duration& server_status_timeout = ros::Duration(5,0)) : nh_(nh, name)
  {
    // Initialize all One Shot Timers
    ack_timer_.registerOneShotCb(boost::bind(&ActionClientT::ackTimeoutCallback, this, _1));
    runtime_timer_.registerOneShotCb(boost::bind(&ActionClientT::runtimeTimeoutCallback, this, _1));
    wait_for_preempted_timer_.registerOneShotCb(boost::bind(&ActionClientT::waitForPreemptedTimeoutCallback, this, _1));
    comm_sync_timer_.registerOneShotCb(boost::bind(&ActionClientT::commSyncTimeoutCallback, this, _1));

    setState(IDLE);
    expecting_result_ = expecting_result;

    goal_pub_    = nh_.advertise<ActionGoal> ("goal", 1);
    preempt_pub_ = nh_.advertise<Preempt> ("preempt", 1);
    status_sub_  = nh_.subscribe("status", 1, &ActionClientT::statusCallback, this);
    result_sub_  = nh_.subscribe("result", 1, &ActionClientT::resultCallback, this);

    server_status_timeout_ = server_status_timeout;
    startServerStatusTimer();
  }

  void execute(const Goal& goal,
               CompletionCallback completion_callback          = CompletionCallback(),
               const ros::Duration& runtime_timeout            = ros::Duration(0,0),
               AckTimeoutCallback ack_timeout_callback         = AckTimeoutCallback(),
               PreemptTimeoutCallback preempt_timeout_callback = PreemptTimeoutCallback(),
               const ros::Duration& ack_timeout                = ros::Duration(5,0),
               const ros::Duration& wait_for_preempted_timeout = ros::Duration(5,0),
               const ros::Duration& comm_sync_timeout          = ros::Duration(5,0))
  {
    ROS_DEBUG("Got call the execute()");
    boost::mutex::scoped_lock(client_state_mutex_);

    if (client_state_ == SERVER_INACTIVE)
    {
      ROS_ERROR("Trying to send an execute command to an inactive server");
      return;
    }

    terminal_status_ = TerminalStatuses::TIMED_OUT;
    cur_goal_.header.stamp = ros::Time::now();
    cur_goal_.goal_id.id  = cur_goal_.header.stamp;
    cur_goal_.goal = goal;
    result_.reset();
    goal_pub_.publish(cur_goal_);
    setState(WAITING_FOR_ACK);
    runtime_timeout_ = runtime_timeout;
    wait_for_preempted_timeout_ = wait_for_preempted_timeout;
    comm_sync_timeout_ = comm_sync_timeout;
    completion_callback_ = completion_callback;
    ack_timeout_callback_ = ack_timeout_callback;
    preempt_timeout_callback_ = preempt_timeout_callback;

    // don't set an ACK timeout for the special case: duration==0
    if (ack_timeout == ros::Duration(0,0))
      ROS_DEBUG("Not setting a timeout for ACK");
    else
    {
      // Set/reset the timeout for WAITING_FOR_GOAL_ACK
      ROS_DEBUG("Starting the [%.2fs] timer for ACK timeout callback", ack_timeout.toSec());
      ack_timer_ = nh_.createTimer(ack_timeout, ack_timer_.getCb());
    }
  }

  TerminalStatuses::TerminalStatus waitUntilDone(ResultConstPtr& result)
  {
    ros::Duration sleep_duration = ros::Duration().fromSec(.1);
    while(true)
    {
      {
        boost::mutex::scoped_lock(client_state_mutex_);
        if (client_state_ == IDLE)
        {
          if (expecting_result_ && result_)
          {
            EnclosureDeleter<const ActionResult> d(result_);
            result = ResultConstPtr(&(result_->result), d);
          }
          else
            result = ResultConstPtr();
          return terminal_status_;
        }
        else if (client_state_ == SERVER_INACTIVE)
        {
          if (expecting_result_ && result_)
          {
            EnclosureDeleter<const ActionResult> d(result_);
            result = ResultConstPtr(&(result_->result), d);
          }
          else
            result = ResultConstPtr();
          return TerminalStatuses::TIMED_OUT;
        }
      }
      sleep_duration.sleep();
    }
  }

private:

  boost::recursive_mutex client_state_mutex_;
  // ***** Lockset for client_state_mutex_ *****

  enum ClientState {SERVER_INACTIVE,IDLE, WAITING_FOR_ACK, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_TERMINAL_STATE, WAITING_FOR_RESULT };
  ClientState client_state_;
  bool server_active_;
  ActionGoal cur_goal_;
  TerminalStatuses::TerminalStatus terminal_status_;
  ros::Duration runtime_timeout_;            // Maximum time we're willing to stay in {PURSUING_GOAL, WAITING_FOR_TERMINAL_STATE, WAITING_FOR_RESULT} before Preempting
  ros::Duration wait_for_preempted_timeout_; // Maximum time we're willing to stay in WAITING_FOR_PREEMPTED until we release the goal as TIMED_OUT
  ros::Duration comm_sync_timeout_;          // Maximum time we're willing to stay in WAITING_FOR_TERMINAL_STATE or WAITING_FOR_RESULT until we release the goal as TIMED_OUT
  bool expecting_result_;
  boost::shared_ptr<const ActionResult> result_;
  CompletionCallback completion_callback_;
  AckTimeoutCallback ack_timeout_callback_;
  PreemptTimeoutCallback preempt_timeout_callback_;

  // *******************************************

  //boost::recursive_mutex completion_cb_mutex_;

  // Various Timers
  OneShotTimer  ack_timer_;                //!< Tracks timeout in WAITING_FOR_ACK state
  OneShotTimer  runtime_timer_;            //!< Tracks timeout in PURSUING_GOAL state
  OneShotTimer  wait_for_preempted_timer_; //!< Tracks timeout in WAITING_FOR_PREEMPTED state
  OneShotTimer  comm_sync_timer_;          //!< Tracks timeout in WAITING_FOR_RESULT or WAITING_FOR_TERMINAL_STATE
  ros::Timer    server_status_timer_;      //!< Forces client_state_ into SERVER_INACTIVE upon not getting status msgs.
  ros::Duration server_status_timeout_;    //!< Duration before we decide that the server has 'died'

  ros::NodeHandle nh_;
  ros::Publisher goal_pub_;
  ros::Publisher preempt_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  void ackTimeoutCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if ( client_state_ == WAITING_FOR_ACK )
    {
      ROS_WARN("Timed out waiting for ACK");
      terminal_status_ = TerminalStatuses::IGNORED;
      if (ack_timeout_callback_)
        ack_timeout_callback_();
      //completion_callback_(TerminalStatuses::IGNORED, ResultConstPtr());
      //setState(IDLE);
    }
  }

  void runtimeTimeoutCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if ( client_state_ == PURSUING_GOAL )
    {
      ROS_WARN("Timed out waiting to finish PURSUING_GOAL");
      preemptGoal();
    }
  }

  void waitForPreemptedTimeoutCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if ( client_state_ == WAITING_FOR_PREEMPTED )
    {
      ROS_WARN("Timed out waiting to finish WAITING_FOR_PREEMPTED");
      setState(IDLE);
      terminal_status_ = TerminalStatuses::TIMED_OUT;
      if (preempt_timeout_callback_)
        preempt_timeout_callback_();
      //if (completion_callback_)
      //  completion_callback_(TerminalStatuses::TIMED_OUT, ResultConstPtr());
    }
  }

  /**
   * Handle the case when we've spent too much time in WAITING_FOR_RESULT or
   * WAITING_FOR_TERMINAL_STATE. We've timed out, but at least call the
   * completion callback with as much information as we've got.
   */
  void commSyncTimeoutCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if (client_state_ == WAITING_FOR_TERMINAL_STATE)
    {
      if (!result_)
        ROS_ERROR("BUG: If we're waiting for a terminal state, then result_ MUST exist");

      setState(IDLE);
      terminal_status_ = TerminalStatuses::TIMED_OUT;
      if (completion_callback_)
      {
        EnclosureDeleter<const ActionResult> d(result_);
        completion_callback_(TerminalStatuses::TIMED_OUT, ResultConstPtr(&(result_->result), d));
      }
    }
    else if (client_state_ == WAITING_FOR_RESULT)
    {
      setState(IDLE);
      terminal_status_ = TerminalStatuses::TIMED_OUT;
      if (completion_callback_)
        completion_callback_(TerminalStatuses::TIMED_OUT, ResultConstPtr());
    }
  }


  void serverStatusTimeoutCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    if (client_state_ != SERVER_INACTIVE)
    {
      ROS_WARN("Timed out waiting on status pings from ActionServer for [%.2fs]. Assuming server is inactive", server_status_timeout_.toSec());
      setState(SERVER_INACTIVE);
    }
  }

  void gotStatusPing()
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    startServerStatusTimer();
    if (client_state_ == SERVER_INACTIVE)
    {
      ROS_INFO("Started receiving status pings from ActionServer again");
      setState(IDLE);
    }
  }

  void startServerStatusTimer()
  {
    server_status_timer_ = nh_.createTimer(server_status_timeout_, boost::bind(&ActionClientT::serverStatusTimeoutCallback, this, _1));
  }

  void preemptGoal()
  {
    ROS_DEBUG("About to preemptGoal()");
    boost::mutex::scoped_lock(client_state_mutex_);
    if (client_state_ == PURSUING_GOAL || client_state_ == WAITING_FOR_ACK)
    {
      Preempt preempt;
      preempt.goal_id = cur_goal_.goal_id;
      preempt.header.stamp = cur_goal_.goal_id.id;
      preempt_pub_.publish(preempt);
      if (wait_for_preempted_timeout_ != ros::Duration(0,0))
      {
        ROS_DEBUG("Starting the [%.2fs] timer for the WAIT_FOR_PREEMPTED timeout", wait_for_preempted_timeout_.toSec());
        wait_for_preempted_timer_ = nh_.createTimer(wait_for_preempted_timeout_, wait_for_preempted_timer_.getCb());
      }
      else
        ROS_DEBUG("Infinte timeout for WAIT_FOR_PREEMPTED timeout");
      setState(WAITING_FOR_PREEMPTED);
    }
    else
      ROS_DEBUG("Not in a preemptable state (ClientState=%u)", client_state_);
  }

  void resultCallback(const ActionResultConstPtr& msg)
  {
    boost::mutex::scoped_lock(client_state_mutex_);

    if (client_state_ == IDLE || client_state_ == SERVER_INACTIVE)
      return;

    if (isCurrentGoal(msg->goal_id))
    {
      ROS_DEBUG("Got a result msg for this goal id");
      if (!expecting_result_)
      {
        ROS_WARN("ActionClient was told to not expect result msgs, yet we still got one");
        return;
      }

      switch(client_state_)
      {
        case WAITING_FOR_ACK:
          ack_timer_.stop();
          goToWaitingForTerminalState(msg);
          break;
        case PURSUING_GOAL:
          runtime_timer_.stop();
          goToWaitingForTerminalState(msg);
          break;
        case WAITING_FOR_PREEMPTED:
          wait_for_preempted_timer_.stop();
          goToWaitingForTerminalState(msg);
          break;
        case WAITING_FOR_RESULT:
          setState(IDLE);
          if (completion_callback_)
          {
            EnclosureDeleter<const ActionResult> d(msg);
            completion_callback_(terminal_status_, ResultConstPtr(&(msg->result), d));
          }
          break;
        case WAITING_FOR_TERMINAL_STATE:
          ROS_WARN("BUG: Got a 2nd ActionResult for the same goal");
          break;
        default:
          ROS_WARN("BUG: Should not ever get to this code");
          break;
      }
    }
  }

  void goToWaitingForTerminalState(const ActionResultConstPtr& result)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    setState(WAITING_FOR_TERMINAL_STATE);
    result_ = result;
    ROS_DEBUG("Starting the [%.2fs] timer for the WAITING_FOR_TERMINAL_STATE (CommunicationSync) timeout", comm_sync_timeout_.toSec());
    comm_sync_timer_ = nh_.createTimer(comm_sync_timeout_, comm_sync_timer_.getCb());
  }

  void statusCallback(const GoalStatusConstPtr& msg)
  {
    boost::mutex::scoped_lock(client_state_mutex_);

    // ***** ADD STATUS PING ****
    gotStatusPing();

    // Don't do any processing on idle messages
    if (msg->status == GoalStatus::IDLE)
      return;

    // Don't care about status if we're not even trying for a goal
    if (client_state_ == IDLE || client_state_ == SERVER_INACTIVE )
      return;

    if ( isFutureGoal(msg->goal_id) )
    {
      if (msg->status != GoalStatus::REJECTED)
      {
        ROS_DEBUG("Saw a future goal. Therefore, our goal somehow got lost during execution");
        setState(IDLE);
        terminal_status_ = TerminalStatuses::LOST;
        // Save the callback that we want to call, and call it later (outside the lock).
        if (completion_callback_)
          completion_callback_(TerminalStatuses::LOST, ResultConstPtr());
      }
      else
      {
        ROS_DEBUG("Saw a future goal be Rejected. Ignoring it, since we're ok with rejected future goals");
      }
    }
    else if( isCurrentGoal(msg->goal_id) )
    {
      if (client_state_ == WAITING_FOR_ACK)
      {
        switch (msg->status)
        {
          case GoalStatus::ACTIVE :
            ack_timer_.stop();
            setState(PURSUING_GOAL);
            if (runtime_timeout_ != ros::Duration(0,0))
            {
              ROS_DEBUG("Starting the [(%.2fs] timer for the PURSUING_GOAL timeout", runtime_timeout_.toSec());
              runtime_timer_ = nh_.createTimer(runtime_timeout_, runtime_timer_.getCb());
            }
            else
              ROS_DEBUG("Infinte timeout for PURSUING_GOAL timeout");
            break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            goToTerminalState(msg->status); break;
          default:
            ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
        }
      }
      else if (client_state_ == PURSUING_GOAL)
      {
        switch (msg->status)
        {
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            runtime_timer_.stop();
            goToTerminalState(msg->status); break;
          default:
            ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
        }
      }
      else if (client_state_ == WAITING_FOR_PREEMPTED)
      {
        switch (msg->status)
        {
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            wait_for_preempted_timer_.stop();
            goToTerminalState(msg->status); break;
          default:
            ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
        }
      }
      else if (client_state_ == WAITING_FOR_TERMINAL_STATE)
      {
        switch (msg->status)
        {
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            goToTerminalState(msg->status); break;
          case GoalStatus::ACTIVE : break;
          default:
            ROS_DEBUG("Not sure how to handle State: %u", msg->status); break;
        }
      }
      else if (client_state_ == WAITING_FOR_RESULT)
      {
        // Do nothing if we're already waiting for the result
      }
      else
      {
        ROS_DEBUG("In ClientState [%u]. Got goal status [%u], Ignoring it", client_state_, msg->status);
      }
    }
  }

  bool isFutureGoal(const GoalID& goal_id)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    return goal_id.id > cur_goal_.goal_id.id;
  }

  bool isCurrentGoal(const GoalID& goal_id)
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    return goal_id.id == cur_goal_.goal_id.id;
  }

  void goToTerminalState(uint8_t status)
  {
    boost::mutex::scoped_lock(client_state_mutex_);

    if(expecting_result_ && !result_)
    {
      switch(status)
      {
        case GoalStatus::PREEMPTED:
          terminal_status_ = TerminalStatuses::PREEMPTED;
          goToWaitingForResult();
          break;
        case GoalStatus::SUCCEEDED:
          terminal_status_ = TerminalStatuses::SUCCEEDED;
          goToWaitingForResult();
          break;
        case GoalStatus::ABORTED:
          terminal_status_ = TerminalStatuses::ABORTED;
          goToWaitingForResult();
          break;
        case GoalStatus::REJECTED:            // We don't wait for a result if we rejected a goal
          setState(IDLE);
          terminal_status_ = TerminalStatuses::REJECTED;
          if (completion_callback_)
            completion_callback_(TerminalStatuses::REJECTED, ResultConstPtr());
          break;
        default:
          ROS_WARN("BUG: Tried to go to a terminal status without receiving a terminal status. [GoalStatus.status==%u]", status);
          break;
      }
    }
    else
    {
      ResultConstPtr unwrapped_result;
      if (expecting_result_ && result_)
      {
        EnclosureDeleter<const ActionResult> d(result_);
        unwrapped_result = ResultConstPtr(&(result_->result), d);
      }
      else
        unwrapped_result.reset(); // If they weren't expecting a result or we never got one, then don't give them one

      switch(status)
      {
        case GoalStatus::PREEMPTED:
          setState(IDLE);
          terminal_status_ = TerminalStatuses::PREEMPTED;
          if (completion_callback_)
            completion_callback_(TerminalStatuses::PREEMPTED, unwrapped_result);
          break;
        case GoalStatus::SUCCEEDED:
          setState(IDLE);
          terminal_status_ = TerminalStatuses::SUCCEEDED;
          if (completion_callback_)
            completion_callback_(TerminalStatuses::SUCCEEDED, unwrapped_result);
          break;
        case GoalStatus::ABORTED:
          terminal_status_ = TerminalStatuses::ABORTED;
          setState(IDLE);
          if (completion_callback_)
            completion_callback_(TerminalStatuses::ABORTED, unwrapped_result);
          break;
        case GoalStatus::REJECTED:
          setState(IDLE);
          terminal_status_ = TerminalStatuses::REJECTED;
          if (completion_callback_)
            completion_callback_(TerminalStatuses::REJECTED, unwrapped_result);
          break;
        default:
          ROS_WARN("BUG: Tried to go to a terminal status without receiving a terminal status. [GoalStatus.status==%u]", status);
          break;
      }
    }
    return;
  }

  void goToWaitingForResult()
  {
    boost::mutex::scoped_lock(client_state_mutex_);
    setState(WAITING_FOR_RESULT);
    ROS_DEBUG("Starting the [%.2fs] timer for the WAITING_FOR_RESULT (CommunicationSync) timeout", comm_sync_timeout_.toSec());
    comm_sync_timer_ = nh_.createTimer(comm_sync_timeout_, comm_sync_timer_.getCb());
  }

};

}

#endif // ACTION_TOOLS_ACTION_CLIENT_H_
