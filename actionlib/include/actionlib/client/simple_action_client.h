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

#ifndef ACTIONLIB_SIMPLE_ACTION_CLIENT_H_
#define ACTIONLIB_SIMPLE_ACTION_CLIENT_H_

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/terminal_state.h"

namespace actionlib
{

/**
 * \brief A Simple client implementation of the ActionInterface which supports only one goal at a time
 *
 * The SimpleActionClient wraps the exisitng ActionClient, and exposes a limited set of easy-to-use hooks
 * for the user. Note that the concept of GoalHandles has been completely hidden from the user, and that
 * they must query the SimplyActionClient directly in order to monitor a goal.
 */
template <class ActionSpec>
class SimpleActionClient
{
private:
  ACTION_DEFINITION(ActionSpec);
  typedef ClientGoalHandle<ActionSpec> GoalHandleT;
  typedef SimpleActionClient<ActionSpec> SimpleActionClientT;

public:
  typedef boost::function<void (const TerminalState& terminal_state, const ResultConstPtr& result) > SimpleDoneCallback;
  typedef boost::function<void () > SimpleActiveCallback;
  typedef boost::function<void (const FeedbackConstPtr& feedback) > SimpleFeedbackCallback;

  /**
   * \brief Simple constructor
   *
   * Constructs a SingleGoalActionClient and sets up the necessary ros topics for the ActionInterface
   * \param name The action name. Defines the namespace in which the action communicates
   * \param spin_thread If true, spins up a thread to service this action's subscriptions. If false,
   *                    then the user has to call ros::spin().
   */
  SimpleActionClient(const std::string& name, bool spin_thread = false) : cur_simple_state_(SimpleGoalState::PENDING)
  {
    initSimpleClient(nh_, name, spin_thread);
  }

  /**
   * \brief Constructor with namespacing options
   *
   * Constructs a SingleGoalActionClient and sets up the necessary ros topics for
   * the ActionInterface, and namespaces them according the a specified NodeHandle
   * \param n The node handle on top of which we want to namespace our action
   * \param name The action name. Defines the namespace in which the action communicates
   * \param spin_thread If true, spins up a thread to service this action's subscriptions. If false,
   *                    then the user has to call ros::spin().
   */
  SimpleActionClient(ros::NodeHandle& n, const std::string& name, bool spin_thread = false) : cur_simple_state_(SimpleGoalState::PENDING)
  {
    initSimpleClient(n, name, spin_thread);

  }

  ~SimpleActionClient();

  /**
   * \brief Blocks until the action server connects to this client
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the server connected in the allocated time. False on timeout
   */
  bool waitForActionServerToStart(const ros::Duration& timeout = ros::Duration(0,0) ) { return ac_->waitForActionServerToStart(timeout); }

  /**
   * \brief Sends a goal to the ActionServer, and also registers callbacks
   *
   * If a previous goal is already active when this is called. We simply forget
   * about that goal and start tracking the new goal. Not cancel requests are made.
   * \param done_cb     Callback that gets called on transitions to Done
   * \param active_cb   Callback that gets called on transitions to Active
   * \param feedback_cb Callback that gets called whenever feedback for this goal is received
   */
  void sendGoal(const Goal& goal,
                SimpleDoneCallback     done_cb     = SimpleDoneCallback(),
                SimpleActiveCallback   active_cb   = SimpleActiveCallback(),
                SimpleFeedbackCallback feedback_cb = SimpleFeedbackCallback());

  /**
   * \brief Blocks until this goal transitions to done
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the goal finished. False if the goal didn't finish within the allocated timeout
   */
  bool waitForGoalToFinish(const ros::Duration& timeout = ros::Duration(0,0) );

  /**
   * \brief Get the current state of the goal: [PENDING], [ACTIVE], or [DONE]
   */
  SimpleGoalState getGoalState();

  /**
   * \brief Get the Result of the current goal
   * \return shared pointer to the result. Note that this pointer will NEVER be NULL
   */
  //ResultConstPtr getResult();
  ResultConstPtr getResult();


  /**
   * \brief Get the terminal state information for this goal
   *
   * Possible States Are: RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
   * This call only makes sense if SimpleGoalState==DONE. This will sends ROS_WARNs if we're not in DONE
   * \return The terminal state
   */
  TerminalState getTerminalState();

  /**
   * \brief Cancel all goals currently running on the action server
   *
   * This preempts all goals running on the action server at the point that
   * this message is serviced by the ActionServer.
   */
  void cancelAllGoals();

  /**
   * \brief Cancel all goals that were stamped at and before the specified time
   * \param time All goals stamped at or before `time` will be canceled
   */
  void cancelGoalsAtAndBeforeTime(const ros::Time& time);

  /**
   * \brief Cancel the goal that we are currently pursuing
   */
  void cancelGoal();

  /**
   * \brief Stops tracking the state of the current goal. Unregisters this goal's callbacks
   *
   * This is useful if we want to make sure we stop calling our callbacks before sending a new goal.
   * Note that this does not cancel the goal, it simply stops looking for status info about this goal.
   */
  void stopTrackingGoal();

private:
  typedef ActionClient<ActionSpec> ActionClientT;
  ros::NodeHandle nh_;
  boost::scoped_ptr<ActionClientT> ac_;
  GoalHandleT gh_;

  SimpleGoalState cur_simple_state_;

  // Signalling Stuff
  boost::condition done_condition_;
  boost::mutex done_mutex_;

  // User Callbacks
  SimpleDoneCallback done_cb_;
  SimpleActiveCallback active_cb_;
  SimpleFeedbackCallback feedback_cb_;

  // Spin Thread Stuff
  boost::mutex terminate_mutex_;
  bool need_to_terminate_;
  boost::thread* spin_thread_;
  ros::CallbackQueue callback_queue;

  // ***** Private Funcs *****
  void initSimpleClient(ros::NodeHandle& n, const std::string& name, bool spin_thread);
  void handleTransition(GoalHandleT gh);
  void handleFeedback(GoalHandleT gh, const FeedbackConstPtr& feedback);
  void setSimpleState(const SimpleGoalState::StateEnum& next_state);
  void setSimpleState(const SimpleGoalState& next_state);
  void spinThread();
};



template<class ActionSpec>
void SimpleActionClient<ActionSpec>::initSimpleClient(ros::NodeHandle& n, const std::string& name, bool spin_thread)
{
  if (spin_thread)
  {
    ROS_DEBUG("Spinning up a thread for the SimpleActionClient");
    need_to_terminate_ = false;
    spin_thread_ = new boost::thread(boost::bind(&SimpleActionClient<ActionSpec>::spinThread, this));
    ac_.reset(new ActionClientT(n, name, &callback_queue));
  }
  else
  {
    spin_thread_ = NULL;
    ac_.reset(new ActionClientT(n, name));
  }
}

template<class ActionSpec>
SimpleActionClient<ActionSpec>::~SimpleActionClient()
{
  if (spin_thread_)
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      need_to_terminate_ = true;
    }
    spin_thread_->join();
    delete spin_thread_;
  }
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::spinThread()
{
  while (nh_.ok())
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      if (need_to_terminate_)
        break;
    }
    callback_queue.callAvailable();
  }
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::setSimpleState(const SimpleGoalState::StateEnum& next_state)
{
  setSimpleState( SimpleGoalState(next_state) );
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::setSimpleState(const SimpleGoalState& next_state)
{
  ROS_DEBUG("Transitioning SimpleState from [%s] to [%s]",
            cur_simple_state_.toString().c_str(),
            next_state.toString().c_str());
  cur_simple_state_ = next_state;
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::sendGoal(const Goal& goal,
                                              SimpleDoneCallback     done_cb,
                                              SimpleActiveCallback   active_cb,
                                              SimpleFeedbackCallback feedback_cb)
{
  // Reset the old GoalHandle, so that our callbacks won't get called anymore
  gh_.reset();

  // Store all the callbacks
  done_cb_     = done_cb;
  active_cb_   = active_cb;
  feedback_cb_ = feedback_cb;

  cur_simple_state_ = SimpleGoalState::PENDING;

  // Send the goal to the ActionServer
  gh_ = ac_->sendGoal(goal, boost::bind(&SimpleActionClientT::handleTransition, this, _1),
                            boost::bind(&SimpleActionClientT::handleFeedback, this, _1, _2));
}

template<class ActionSpec>
SimpleGoalState SimpleActionClient<ActionSpec>::getGoalState()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to getGoalState() when no goal is running. You are incorrectly using SimpleActionClient");

  CommState comm_state_ = gh_.getCommState();

  switch( comm_state_.state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
    case CommState::PENDING:
    case CommState::RECALLING:
      return SimpleGoalState(SimpleGoalState::PENDING);
    case CommState::ACTIVE:
    case CommState::PREEMPTING:
      return SimpleGoalState(SimpleGoalState::ACTIVE);
    case CommState::DONE:
      return SimpleGoalState(SimpleGoalState::DONE);
    case CommState::WAITING_FOR_RESULT:
    case CommState::WAITING_FOR_CANCEL_ACK:
      return cur_simple_state_;
    default:
      break;
  }
  ROS_ERROR("Error trying to interpret CommState - %u", comm_state_.state_);
  return SimpleGoalState(SimpleGoalState::DONE);
}

template<class ActionSpec>
TerminalState SimpleActionClient<ActionSpec>::getTerminalState()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to getTerminalState() when no goal is running. You are incorrectly using SimpleActionClient");

  if (gh_.getCommState() != CommState::DONE)
    ROS_ERROR("Trying to getTerminalState() when we're not yet in a [DONE] state. You are incorrectly using SimpleActionClient");

  return gh_.getTerminalState();
}

template<class ActionSpec>
typename SimpleActionClient<ActionSpec>::ResultConstPtr SimpleActionClient<ActionSpec>::getResult()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to getResult() when no goal is running. You are incorrectly using SimpleActionClient");

  if (gh_.getResult())
    return gh_.getResult();

  return ResultConstPtr(new Result);
}


template<class ActionSpec>
void SimpleActionClient<ActionSpec>::cancelAllGoals()
{
  ac_->cancelAllGoals();
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::cancelGoalsAtAndBeforeTime(const ros::Time& time)
{
  ac_->cancelAllGoalsBeforeTime(time);
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::cancelGoal()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to cancelGoal() when no goal is running. You are incorrectly using SimpleActionClient");

  gh_.cancel();
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::stopTrackingGoal()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to stopTrackingGoal() when no goal is running. You are incorrectly using SimpleActionClient");
  gh_.reset();
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::handleFeedback(GoalHandleT gh, const FeedbackConstPtr& feedback)
{
  if (gh_ != gh)
    ROS_ERROR("Got a callback on a goalHandle that we're not tracking.  \
               This is an internal SimpleActionClient/ActionClient bug.  \
               This could also be a GoalID collision");
  if (feedback_cb_)
    feedback_cb_(feedback);
}

template<class ActionSpec>
void SimpleActionClient<ActionSpec>::handleTransition(GoalHandleT gh)
{
  CommState comm_state_ = gh.getCommState();
  switch (comm_state_.state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
      ROS_ERROR("BUG: Shouldn't ever get a transition callback for WAITING_FOR_GOAL_ACK");
      break;
    case CommState::PENDING:
      ROS_ERROR_COND( cur_simple_state_ != SimpleGoalState::PENDING,
                      "BUG: Got a transition to CommState [%s] when our in SimpleGoalState [%s]",
                      comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
      break;
    case CommState::ACTIVE:
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
          setSimpleState(SimpleGoalState::ACTIVE);
          if (active_cb_)
            active_cb_();
          break;
        case SimpleGoalState::ACTIVE:
          break;
        case SimpleGoalState::DONE:
          ROS_ERROR("BUG: Got a transition to CommState [%s] when in SimpleGoalState [%s]",
                    comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
          break;
        default:
          ROS_FATAL("Unknown SimpleGoalState %u", cur_simple_state_.state_);
          break;
      }
      break;
    case CommState::WAITING_FOR_RESULT:
      break;
    case CommState::WAITING_FOR_CANCEL_ACK:
      break;
    case CommState::RECALLING:
      ROS_ERROR_COND( cur_simple_state_ != SimpleGoalState::PENDING,
                      "BUG: Got a transition to CommState [%s] when our in SimpleGoalState [%s]",
                      comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
      break;
    case CommState::PREEMPTING:
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
          setSimpleState(SimpleGoalState::ACTIVE);
          if (active_cb_)
            active_cb_();
          break;
        case SimpleGoalState::ACTIVE:
          break;
        case SimpleGoalState::DONE:
          ROS_ERROR("BUG: Got a transition to CommState [%s] when in SimpleGoalState [%s]",
                     comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
          break;
        default:
          ROS_FATAL("Unknown SimpleGoalState %u", cur_simple_state_.state_);
          break;
      }
      break;
    case CommState::DONE:
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
        case SimpleGoalState::ACTIVE:
          done_mutex_.lock();
          setSimpleState(SimpleGoalState::DONE);
          done_mutex_.unlock();

          if (done_cb_)
            done_cb_(gh.getTerminalState(), gh.getResult());

          done_condition_.notify_all();
          break;
        case SimpleGoalState::DONE:
          ROS_ERROR("BUG: Got a second transition to DONE");
          break;
        default:
          ROS_FATAL("Unknown SimpleGoalState %u", cur_simple_state_.state_);
          break;
      }
      break;
    default:
      ROS_ERROR("Unknown CommState received [%u]", comm_state_.state_);
      break;
  }
}

template<class ActionSpec>
bool SimpleActionClient<ActionSpec>::waitForGoalToFinish(const ros::Duration& timeout )
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to waitForGoalToFinish() when no goal is running. You are incorrectly using SimpleActionClient");

  if (timeout < ros::Duration(0,0))
    ROS_WARN("Timeouts can't be negative. Timeout is [%.2fs]", timeout.toSec());

  ros::Time timeout_time = ros::Time::now() + timeout;

  boost::mutex::scoped_lock lock(done_mutex_);

  // Hardcode how often we check for node.ok()
  ros::Duration loop_period = ros::Duration().fromSec(.1);

  while (nh_.ok())
  {
    // Determine how long we should wait
    ros::Duration time_left = timeout_time - ros::Time::now();

    // Check if we're past the timeout time
    if (timeout != ros::Duration(0,0) && time_left <= ros::Duration(0,0) )
      break;

    if (cur_simple_state_ == SimpleGoalState::DONE)
      break;

    // Truncate the time left
    if (time_left > loop_period)
      time_left = loop_period;

    done_condition_.timed_wait(lock, boost::posix_time::milliseconds(time_left.toSec() * 1000.0f));
  }

  return (cur_simple_state_ == SimpleGoalState::DONE);
}


}

#endif // ACTIONLIB_SINGLE_GOAL_ACTION_CLIENT_H_
