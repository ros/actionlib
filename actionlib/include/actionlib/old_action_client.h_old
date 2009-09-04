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

#ifndef ACTIONLIB_OLD_ACTION_CLIENT_H_
#define ACTIONLIB_OLD_ACTION_CLIENT_H_

#include <list>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "actionlib/one_shot_timer.h"
#include "actionlib/goal_id_generator.h"
#include "actionlib/client_goal_status.h"
#include "actionlib/one_shot_timer.h"
#include "actionlib/enclosure_deleter.h"

// Messages
//#include "actionlib/ActionHeader.h"
#include "actionlib/GoalStatusArray.h"
#include "actionlib/RequestType.h"

namespace actionlib
{

//! \todo figure out why I get compile errors trying to use boost::mutex::scoped_lock()
class ScopedLock
{
public:
  ScopedLock(boost::recursive_mutex& mutex) : mutex_(mutex)
  {
    mutex_.lock();
  }
  ~ScopedLock()
  {
    mutex_.unlock();
  }
private:
  boost::recursive_mutex& mutex_;

};


template<class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
class ActionClient
{
public:
  // Need forward declaration for typedefs
  class GoalHandle;
private:
  typedef ActionClient<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback> ActionClientT;
  typedef boost::shared_ptr<const ActionGoal> ActionGoalConstPtr;
  typedef boost::shared_ptr<const Goal> GoalConstPtr;

  typedef boost::shared_ptr<const ActionResult> ActionResultConstPtr;
  typedef boost::shared_ptr<const Result> ResultConstPtr;

  typedef boost::shared_ptr<const ActionFeedback> ActionFeedbackConstPtr;
  typedef boost::shared_ptr<const Feedback> FeedbackConstPtr;

  typedef boost::function<void (const GoalHandle&) > CompletionCallback;
  typedef boost::function<void (const GoalHandle&, const FeedbackConstPtr&) > FeedbackCallback;

  class GoalManager
  {
    public:
      //! \brief Builds the goal manager and then sends the goal over the wire
      GoalManager(const GoalID& goal_id, const Goal& goal, CompletionCallback ccb, FeedbackCallback fcb, ActionClientT* ac, const ros::Duration& runtime_timeout);
      void preemptGoal(const ros::Time& preempt_time);
      ClientGoalStatus getStatus();
      ResultConstPtr getResult();

    private:
      enum CommState {WAITING_FOR_ACK, PENDING, PURSUING_GOAL, WAITING_FOR_RESULT, DONE};
      std::string commStateToString(const CommState& state)
      {
        switch (state)
        {
          case WAITING_FOR_ACK:
            return "WAITING_FOR_ACK";
          case PENDING:
            return "PENDING";
          case PURSUING_GOAL:
            return "PURSUING_GOAL";
          case WAITING_FOR_RESULT:
            return "WAITING_FOR_RESULT";
          case DONE:
            return "DONE";
          default:
            ROS_ERROR("Trying to lookup unknown CommState");
            break;
        }
        return "BUG-UNKNOWN";
      }

      void setCommState(const CommState& state);
      void setClientGoalStatus(const ClientGoalStatus::StateEnum& next_status_enum);
      void setClientGoalStatus(const ClientGoalStatus& next_status);
      void updateStatus(const GoalStatusArrayConstPtr& status_array);
      void updateFeedback(const ActionFeedbackConstPtr& feedback);
      void updateResult(const ActionResultConstPtr& result);

      void startWaitingForAck();
      void startPending();
      void startPursuingGoal();
      void startWaitingForResult(const ClientGoalStatus& next_client_goal_status);
      void startDone(const ActionResultConstPtr& result);
      void processLost();               // No explicit state transition, but looks like a state transition

      void finishWaitingForAck();
      void finishPending();
      void finishPursuingGoal();
      void finishWaitingForResult();

      void waitingForAckTimeoutCallback(const ros::TimerEvent& e);
      void runtimeTimeoutCallback(const ros::TimerEvent& e);
      void waitingForResultTimeoutCallback(const ros::TimerEvent& e);

      const GoalStatus* findGoalStatus(const std::vector<GoalStatus>& status_vec);

      //boost::mutex mutex_;
      ClientGoalStatus client_goal_status_;
      CommState comm_state_;                        //!< The internal state if the communication protocol
      GoalID goal_id_;                              //!< ID associated with this goal
      ResultConstPtr result_;

      ActionClientT* ac_;                           //!< The action client we'll use for sending msgs over the wire
      boost::weak_ptr<void> handle_tracker_;        //!< Refcounts the # of goalsHandles for this GoalManager
      typename std::list<boost::shared_ptr<GoalManager> >::iterator it_; //!< Needed to construct a GoalHandle later

      // All the various state timers
      OneShotTimer  waiting_for_ack_timer_;       //!< Tracks timeout in WAITING_FOR_ACK state
      OneShotTimer  runtime_timer_;               //!< Tracks timeout in PURSUING_GOAL state
      OneShotTimer  waiting_for_result_timer_;    //!< Tracks timeout in WAITING_FOR_RESULT or WAITING_FOR_TERMINAL_STATE
      ros::Duration runtime_timeout_;

      CompletionCallback ccb_;
      FeedbackCallback fcb_;

      friend class ActionClient;
  };

  typedef boost::shared_ptr<GoalManager> GoalManagerPtr;

  // Used to clean up the GoalManager list in the ActionClient, once there are no more goal handles pointing to the status
  class HandleTrackerDeleter
  {
    public:
      HandleTrackerDeleter(ActionClientT* ac, typename std::list<GoalManagerPtr>::iterator it) : it_(it), ac_(ac)
      {  }

      void operator() (void* ptr)
      {
        ROS_DEBUG("About to delete a GoalManager");
        ScopedLock(ac_->manager_list_mutex_);
        ac_->manager_list_.erase(it_);
      }

    private:
      typename std::list<GoalManagerPtr>::iterator it_;
      ActionClientT* ac_;
  };

public:

  /**
   * \brief Handle for monitoring and manipulating a specific goal request
   */
  class GoalHandle
  {
    public:
      GoalHandle()
      {
        valid_ = false;
      }

      /**
       * \brief Disconnect handle from an existing goal
       * Normally, a user will wait for a GoalHandle to go out of scope to stop tracking a
       * specific goal, but it could be useful to stop tracking a goal sooner by calling reset()
       */
      void reset()
      {
        valid_ = false;
        status_it_ = typename std::list<GoalManagerPtr>::iterator();
        handle_tracker_.reset();
        ac_ = NULL;
      }

      /**
       * \brief Get the status of this goal
       * \return The status of the current goal
       */
      ClientGoalStatus getStatus() const
      {
        ScopedLock(ac_->manager_list_mutex_);
        return (*status_it_)->getStatus();
      }

      /**
       * \brief Get the result associted with the goal
       * \return Shared pointer to the result. Returns NULL if result doesn't exist
       */
      ResultConstPtr getResult() const
      {
        ScopedLock(ac_->manager_list_mutex_);
        return (*status_it_)->getResult();
      }

      /**
       * \brief Preempt this goal
       * Preempt only this goal OR preempt this goal and all goals that occur before a specific time
       * \param preempt_time Preempt all goals before or at this time. Ignored if 0
       */
      void preemptGoal(ros::Time preempt_time = ros::Time(0,0)) const
      {
        ScopedLock(ac_->manager_list_mutex_);
        (*status_it_)->preemptGoal(preempt_time);
      }

    private:
      GoalHandle(typename std::list<GoalManagerPtr>::iterator it, ActionClientT* ac)
       : valid_(true), status_it_(it), handle_tracker_((*it)->handle_tracker_.lock()), ac_(ac)
      {  }
      bool valid_;
      typename std::list<GoalManagerPtr>::iterator status_it_;
      boost::shared_ptr<void> handle_tracker_;
      ActionClientT* ac_;
      friend class ActionClient;
  };

  ActionClient(const std::string& name) : n_(name), id_generator_(name)
  {
    initClient();
  }

  ActionClient(const ros::NodeHandle& n, const std::string& name) : n_(n, name), id_generator_(name)
  {
    initClient();
  }

  GoalHandle sendGoal(const Goal& goal,
                      CompletionCallback ccb = CompletionCallback(),
                      FeedbackCallback fcb   = FeedbackCallback(),
                      ros::Duration timeout = ros::Duration(0,0) )
  {
    GoalID goal_id = id_generator_.generateID();

    // Add a goal manager to our list of managers

    GoalManagerPtr cur_manager = GoalManagerPtr(new GoalManager(goal_id, goal, ccb, fcb, this, timeout));

    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.insert(manager_list_.end(), cur_manager );

    // Tell the goal manager where it is in the list
    (*it)->it_ = it;

    // Create a custom deallocater to eventually destroy the GoalManager, once
    //   we no longer have any GoalHandles in scope
    HandleTrackerDeleter d(this, it);
    boost::shared_ptr<void> handle_tracker((void*) NULL, d);
    (*it)->handle_tracker_ = handle_tracker;

    GoalHandle gh = GoalHandle(it, this);

    return gh;
  }

  /**
   * \brief Preempt goals on the ActionServer, based on timestamp
   * Preempts all the goals on the action server that occur at or before the specified time. If
   * a time of 0 given, then every goal currently on the action server is preempted.
   * \param preempt_time Time specifying which goals to preempt.  Defaults to 0.
   */
  void preempt(ros::Time preempt_time = ros::Time(0,0))
  {
    ActionGoal preempt_msg;
    preempt_msg.request_type.type = RequestType::PREEMPT_REQUEST;
    preempt_msg.goal_id = GoalID();
    preempt_msg.goal_id.stamp = preempt_time;

    goal_pub_.publish(preempt_msg);
  }

private:
  ros::NodeHandle n_;

  ros::Subscriber feedback_sub_;
  ros::Publisher  goal_pub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  boost::recursive_mutex manager_list_mutex_;
  std::list<GoalManagerPtr > manager_list_;

  GoalIDGenerator id_generator_;

  // Timer durations
  ros::Duration waiting_for_ack_timeout_;       // Maximum time we're willing to stay in WAITING_FOR_ACK
  ros::Duration waiting_for_result_timeout_;    // Maximum time we're willing to stay in WAITING_FOR_RESULT until we release the goal as TIMED_OUT

  // *************** Implementation ***************
  void initClient()
  {
    // Load timeout parameters
    double waiting_for_ack_timeout_secs;
    n_.param("WaitingForAckTimeout", waiting_for_ack_timeout_secs, 5.0);
    waiting_for_ack_timeout_.fromSec(waiting_for_ack_timeout_secs);
    ROS_INFO("WaitingForAckTimeout: [%.2fs]", waiting_for_ack_timeout_.toSec());

    double waiting_for_result_timeout_secs;
    n_.param("WaitingForResultTimeout", waiting_for_result_timeout_secs, 0.0);
    waiting_for_result_timeout_.fromSec(waiting_for_result_timeout_secs);
    ROS_INFO("WaitingForResultTimeout: [%.2fs]", waiting_for_result_timeout_.toSec());

    // Start publishers and subscribers
    goal_pub_ = n_.advertise<ActionGoal>("goal", 1);
    status_sub_   = n_.subscribe("status",   1, &ActionClientT::statusCb, this);
    feedback_sub_ = n_.subscribe("feedback", 1, &ActionClientT::feedbackCb, this);
    result_sub_   = n_.subscribe("result",   1, &ActionClientT::resultCb, this);
  }

  void statusCb(const GoalStatusArrayConstPtr& status_array)
  {
    boost::mutex::scoped_lock(manager_list_mutex_);
    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.begin();
    while(it != manager_list_.end())
    {
      (*it)->updateStatus(status_array);
      ++it;
    }
  }

  void feedbackCb(const ActionFeedbackConstPtr& feedback)
  {
    boost::mutex::scoped_lock(manager_list_mutex_);
    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.begin();
    while(it != manager_list_.end())
    {
      (*it)->updateFeedback(feedback);
      ++it;
    }
  }

  void resultCb(const ActionResultConstPtr& result)
  {
    boost::mutex::scoped_lock(manager_list_mutex_);
    typename std::list<GoalManagerPtr >::iterator it ;
    it = manager_list_.begin();
    while(it != manager_list_.end())
    {
      (*it)->updateResult(result);
      ++it;
    }
  }

};

#define ActionClientTemplate \
  template<class ActionGoal, class Goal, class ActionResult, class Result, class ActionFeedback, class Feedback>
#define ActionClientPrefix \
  ActionClient<ActionGoal, Goal, ActionResult, Result, ActionFeedback, Feedback>

ActionClientTemplate
ActionClientPrefix::GoalManager::GoalManager(const GoalID& goal_id, const Goal& goal,
                                             CompletionCallback ccb, FeedbackCallback fcb,
                                             ActionClientT* ac, const ros::Duration& runtime_timeout)
 : client_goal_status_(ClientGoalStatus::PENDING),
   comm_state_(WAITING_FOR_ACK),
   goal_id_(goal_id),
   ac_(ac),
   runtime_timeout_(runtime_timeout),
   ccb_(ccb),
   fcb_(fcb)
{
  waiting_for_ack_timer_.registerOneShotCb(      boost::bind(&ActionClientPrefix::GoalManager::waitingForAckTimeoutCallback, this, _1));
  runtime_timer_.registerOneShotCb(              boost::bind(&ActionClientPrefix::GoalManager::runtimeTimeoutCallback, this, _1));
  waiting_for_result_timer_.registerOneShotCb(   boost::bind(&ActionClientPrefix::GoalManager::waitingForResultTimeoutCallback, this, _1));

  startWaitingForAck();

  boost::shared_ptr<ActionGoal> action_goal(new ActionGoal);
  action_goal->goal_id = goal_id;
  action_goal->request_type.type = RequestType::GOAL_REQUEST;
  action_goal->goal = goal;
  ac_->goal_pub_.publish(action_goal);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::preemptGoal(const ros::Time& preempt_time)
{
  ActionGoal preempt_msg;
  preempt_msg.request_type.type = RequestType::PREEMPT_REQUEST;
  preempt_msg.goal_id = goal_id_;
  preempt_msg.goal_id.stamp = preempt_time;

  ac_->goal_pub_.publish(preempt_msg);
}

ActionClientTemplate
const GoalStatus* ActionClientPrefix::GoalManager::findGoalStatus(const std::vector<GoalStatus>& status_vec)
{
  for (unsigned int i=0; i<status_vec.size(); i++)
    if (status_vec[i].goal_id.id == goal_id_.id)
      return &status_vec[i];
  return NULL;
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::updateStatus(const GoalStatusArrayConstPtr& status_array)
{
  ScopedLock(ac_->manager_list_mutex_);

  switch( comm_state_ )
  {
    case WAITING_FOR_ACK :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        finishWaitingForAck();
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            startPending();break;
          case GoalStatus::ACTIVE :
            startPursuingGoal();break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          default:
            ROS_ERROR("BUG: Got an unknown status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      break;
    }
    case PENDING :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            break;
          case GoalStatus::ACTIVE :
            finishPending();
            startPursuingGoal(); break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
          case GoalStatus::REJECTED :
            finishPending();
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          default:
            ROS_ERROR("BUG: Got an unknown goal status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      else
        processLost();
      break;
    }
    case PURSUING_GOAL :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            ROS_ERROR("Invalid transition from PURSUING_GOAL to PENDING"); break;
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
          case GoalStatus::SUCCEEDED :
          case GoalStatus::ABORTED :
            ROS_DEBUG("In status update");
            finishPursuingGoal();
            startWaitingForResult(ClientGoalStatus(*goal_status)); break;
          case GoalStatus::REJECTED :
            ROS_ERROR("Invalid transition from PURSUING_GOAL to REJECTED"); break;
          default:
            ROS_ERROR("BUG: Got an unknown goal status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      else
        processLost();
      break;
    }
    case WAITING_FOR_RESULT :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            break;
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
            if (client_goal_status_ != ClientGoalStatus::PREEMPTED)
              ROS_ERROR("Got GoalStatus [PREEMPTED], but we're in ClientGoalState [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::SUCCEEDED :
            if (client_goal_status_ != ClientGoalStatus::SUCCEEDED)
              ROS_ERROR("Got GoalStatus==[SUCCEEDED], but we're in ClientGoalState==[%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::ABORTED :
            if (client_goal_status_ != ClientGoalStatus::ABORTED)
              ROS_ERROR("Got GoalStatus==[ABORTED], but we're in ClientGoalState==[%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::REJECTED :
            ROS_ERROR("Invalid Transition from WAITING_FOR_RESUT to REJECTED"); break;
          default:
            ROS_ERROR("BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      // No processLost() call here, since it's possible that the ActionServer stopped sending status,
      //   but we're still waiting for the result to come over the wire.
      break;
    }
    case DONE :
    {
      const GoalStatus* goal_status = findGoalStatus(status_array->status_list);
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case GoalStatus::PENDING :
            break;
          case GoalStatus::ACTIVE :
            break;
          case GoalStatus::PREEMPTED :
            if (client_goal_status_ != ClientGoalStatus::PREEMPTED)
              ROS_ERROR("Got GoalStatus [PREEMPTED], but we're in ClientGoalStatus [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::SUCCEEDED :
            if (client_goal_status_ != ClientGoalStatus::SUCCEEDED)
              ROS_ERROR("Got GoalStatus [SUCCEEDED], but we're in ClientGoalStatus [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::ABORTED :
            if (client_goal_status_ != ClientGoalStatus::ABORTED)
              ROS_ERROR("Got GoalStatus [ABORTED], but we're in ClientGoalStatus [%s]", client_goal_status_.toString().c_str());
            break;
          case GoalStatus::REJECTED :
            ROS_ERROR("Invalid transition from DONE to REJECTED"); break;
          default:
            ROS_ERROR("BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      break;
    }
    default :
      ROS_ERROR("BUG: Unknown CommState. comm_state_=%u", comm_state_);
  }
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::updateFeedback(const ActionFeedbackConstPtr& action_feedback)
{
  ScopedLock(ac_->manager_list_mutex_);

  // Check if this feedback is for us
  if (goal_id_.id != action_feedback->status.goal_id.id)
    return;

  if (fcb_)
  {
    EnclosureDeleter<const ActionFeedback> d(action_feedback);
    FeedbackConstPtr feedback(&(action_feedback->feedback), d);
    GoalHandle gh(it_, ac_);
    fcb_(gh, feedback);
  }
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::updateResult(const ActionResultConstPtr& result)
{
  // {WAITING_FOR_ACK, PENDING, PURSUING_GOAL, WAITING_FOR_PREEMPTED, WAITING_FOR_RESULT, DONE};

  ScopedLock(ac_->manager_list_mutex_);

  // Check if this result is for our goal
  if (goal_id_.id != result->status.goal_id.id)
    return;

  switch( comm_state_ )
  {
    case WAITING_FOR_ACK:
    case PENDING:
    case PURSUING_GOAL:
    case WAITING_FOR_RESULT:
      // Do cleanup for exiting state
      switch (comm_state_)
      {
        case WAITING_FOR_ACK:       finishWaitingForAck(); break;
        case PENDING:               finishPending(); break;
        case PURSUING_GOAL:         finishPursuingGoal(); break;
        case WAITING_FOR_RESULT:    finishWaitingForResult(); break;
        default: ROS_ERROR("BUG: In a funny state"); break;
      }

      switch (result->status.status)
      {
        case GoalStatus::PENDING:
          ROS_ERROR("Got a Result with a PENDING GoalStatus"); break;
        case GoalStatus::ACTIVE:
          ROS_ERROR("Got a Result with an ACTIVE GoalStatus"); break;
        case GoalStatus::PREEMPTED:
        case GoalStatus::SUCCEEDED:
        case GoalStatus::ABORTED:
        case GoalStatus::REJECTED:
          setClientGoalStatus(ClientGoalStatus(result->status));
          startDone(result); break;
        default:
          ROS_ERROR("BUG: Got an unknown status from the ActionServer. status = %u", result->status.status); break;
      }
      break;
    case DONE:
      if (client_goal_status_ == ClientGoalStatus::LOST)
        ROS_WARN("Got a result for a Goal after we thought it was lost");
      else
        ROS_ERROR("Got a second Result for this goal");
      break;
    default:
      ROS_ERROR("BUG: Unknown CommState. comm_state_=%u", comm_state_); break;
  }
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::setClientGoalStatus(const ClientGoalStatus::StateEnum& next_status_enum)
{
  setClientGoalStatus(ClientGoalStatus(next_status_enum));
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::setClientGoalStatus(const ClientGoalStatus& next_status)
{
  ROS_DEBUG("Transitioning ClientGoalStatus from [%s] to [%s]",
            client_goal_status_.toString().c_str(), next_status.toString().c_str());
  client_goal_status_ = next_status;
}

ActionClientTemplate
ClientGoalStatus ActionClientPrefix::GoalManager::getStatus()
{
  switch (comm_state_)
  {
    case WAITING_FOR_ACK:
      return ClientGoalStatus(ClientGoalStatus::PENDING);
    case PENDING:
      return ClientGoalStatus(ClientGoalStatus::PENDING);
    case PURSUING_GOAL:
      return ClientGoalStatus(ClientGoalStatus::ACTIVE);
    case WAITING_FOR_RESULT:
      return ClientGoalStatus(ClientGoalStatus::ACTIVE);
    case DONE:
      return client_goal_status_;
    default:
      ROS_ERROR("BUG - Unknown comm state");
      break;
  }
  return ClientGoalStatus(ClientGoalStatus::LOST);
}

ActionClientTemplate
boost::shared_ptr<const Result> ActionClientPrefix::GoalManager::getResult()
{
  return result_;
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::setCommState(const CommState& next_state)
{
  ROS_DEBUG("Transitioning CommState from %s to %s", commStateToString(comm_state_).c_str(), commStateToString(next_state).c_str());
  comm_state_ = next_state;
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startWaitingForAck()
{
  if (ac_->waiting_for_ack_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the WaitForAck timeout", ac_->waiting_for_ack_timeout_.toSec());

    waiting_for_ack_timer_ = ac_->n_.createTimer(ac_->waiting_for_ack_timeout_, waiting_for_ack_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite WaitForAck timeout");
  setCommState(WAITING_FOR_ACK);
  setClientGoalStatus(ClientGoalStatus::PENDING);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::waitingForAckTimeoutCallback(const ros::TimerEvent& e)
{
  ROS_DEBUG("WaitingForAckTimer Timout Callback");
  ScopedLock(ac_->manager_list_mutex_);
  if (comm_state_ == WAITING_FOR_ACK)
    processLost();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishWaitingForAck()
{
  ROS_DEBUG("Stopping WaitForAck timer");
  waiting_for_ack_timer_.stop();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startPending()
{
  setCommState(PENDING);
  setClientGoalStatus(ClientGoalStatus::PENDING);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishPending()
{

}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startPursuingGoal()
{
  if (runtime_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the Runtime timeout", runtime_timeout_.toSec());
    runtime_timer_ = ac_->n_.createTimer(runtime_timeout_, runtime_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite Runtime timeout");

  setCommState(PURSUING_GOAL);
  setClientGoalStatus(ClientGoalStatus::ACTIVE);
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::runtimeTimeoutCallback(const ros::TimerEvent& e)
{
  ScopedLock(ac_->manager_list_mutex_);
  preemptGoal(ros::Time(0,0));
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishPursuingGoal()
{
  ROS_DEBUG("Stopping Runtime timer");
  runtime_timer_.stop();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startWaitingForResult(const ClientGoalStatus& next_client_goal_status)
{
  if (ac_->waiting_for_result_timeout_ != ros::Duration(0,0))
  {
    ROS_DEBUG("Starting [%.2fs] timer for the WaitForResult timeout", ac_->waiting_for_result_timeout_.toSec());
    waiting_for_result_timer_ = ac_->n_.createTimer(ac_->waiting_for_result_timeout_, waiting_for_result_timer_.getCb());
  }
  else
    ROS_DEBUG("Infinite WaitForResult timeout");

  setCommState(WAITING_FOR_RESULT);
  setClientGoalStatus( ClientGoalStatus(next_client_goal_status) );
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::waitingForResultTimeoutCallback(const ros::TimerEvent& e)
{
  ROS_DEBUG("WaitingForResult Timed out");
  processLost();
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::processLost()
{
  setClientGoalStatus(ClientGoalStatus::LOST);
  startDone(ActionResultConstPtr());
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::startDone(const ActionResultConstPtr& action_result)
{
  if (action_result)
  {
    EnclosureDeleter<const ActionResult> d(action_result);
    result_ = ResultConstPtr(&(action_result->result), d);
  }
  else
    result_.reset(); // If they weren't expecting a result or we never got one, then don't give them one

  setCommState(DONE);


  if (ccb_)
  {
    GoalHandle gh(it_, ac_);
    ccb_(gh);
  }
}

ActionClientTemplate
void ActionClientPrefix::GoalManager::finishWaitingForResult()
{
  ROS_DEBUG("Stopping WaitForResult timer");
  waiting_for_result_timer_.stop();
}

}

#undef ActionClientTemplate
#undef ActionClientPrefix

#endif // ACTIONLIB_OLD_ACTION_CLIENT_H_
