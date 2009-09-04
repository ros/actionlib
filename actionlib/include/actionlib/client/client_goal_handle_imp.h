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

/* This file has the template implementation for ClientGoalHandle. It should be included with the
 * class definition.
 */

namespace actionlib
{

template<class ActionSpec>
ClientGoalHandle<ActionSpec>::ClientGoalHandle()
{
  gm_ = NULL;
  active_ = false;
}

template<class ActionSpec>
ClientGoalHandle<ActionSpec>::ClientGoalHandle(GoalManagerT* gm, typename ManagedListT::Handle handle)
{
  gm_ = gm;
  active_ = true;
  list_handle_ = handle;
}

template<class ActionSpec>
void ClientGoalHandle<ActionSpec>::reset()
{
  list_handle_.reset();
  active_ = false;
  gm_ = NULL;
}

template<class ActionSpec>
bool ClientGoalHandle<ActionSpec>::isExpired() const
{
  return !active_;
}


template<class ActionSpec>
CommState ClientGoalHandle<ActionSpec>::getCommState()
{
  if (!active_)
    ROS_ERROR("Trying to getCommState on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");

  assert(gm_);

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
  return list_handle_.getElem()->getCommState();
}

template<class ActionSpec>
TerminalState ClientGoalHandle<ActionSpec>::getTerminalState()
{
  if (!active_)
    ROS_ERROR("Trying to getTerminalState on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");

  assert(gm_);

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
  CommState comm_state_ = list_handle_.getElem()->getCommState();
  if (comm_state_ != CommState::DONE)
    ROS_WARN("Asking for the terminal state when we're in [%s]", comm_state_.toString().c_str());

  actionlib_msgs::GoalStatus goal_status = list_handle_.getElem()->getGoalStatus();

  switch (goal_status.status)
  {
    case actionlib_msgs::GoalStatus::PENDING:
    case actionlib_msgs::GoalStatus::ACTIVE:
    case actionlib_msgs::GoalStatus::PREEMPTING:
    case actionlib_msgs::GoalStatus::RECALLING:
      ROS_ERROR("Asking for terminal state, but latest goal status is %u", goal_status.status); return TerminalState::LOST;
    case actionlib_msgs::GoalStatus::PREEMPTED: return TerminalState::PREEMPTED;
    case actionlib_msgs::GoalStatus::SUCCEEDED: return TerminalState::SUCCEEDED;
    case actionlib_msgs::GoalStatus::ABORTED:   return TerminalState::ABORTED;
    case actionlib_msgs::GoalStatus::REJECTED:  return TerminalState::REJECTED;
    case actionlib_msgs::GoalStatus::RECALLED:  return TerminalState::RECALLED;
    case actionlib_msgs::GoalStatus::LOST:      return TerminalState::LOST;
    default:
      ROS_ERROR("Unknown goal status: %u", goal_status.status); break;
  }

  ROS_ERROR("Bug in determining terminal state");
  return TerminalState::LOST;
}

template<class ActionSpec>
typename ClientGoalHandle<ActionSpec>::ResultConstPtr ClientGoalHandle<ActionSpec>::getResult()
{
  if (!active_)
    ROS_ERROR("Trying to getResult on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
  assert(gm_);
  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
  return list_handle_.getElem()->getResult();
}

template<class ActionSpec>
void ClientGoalHandle<ActionSpec>::resend()
{
  if (!active_)
    ROS_ERROR("Trying to resend() on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
  assert(gm_);
  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);

  ActionGoalConstPtr action_goal = list_handle_.getElem()->getActionGoal();

  if (!action_goal)
    ROS_ERROR("BUG: Got a NULL action_goal");

  if (gm_->send_goal_func_)
    gm_->send_goal_func_(action_goal);
}

template<class ActionSpec>
void ClientGoalHandle<ActionSpec>::cancel()
{
  if (!active_)
    ROS_ERROR("Trying to cancel() on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
  assert(gm_);
  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);

  ActionGoalConstPtr action_goal = list_handle_.getElem()->getActionGoal();

  actionlib_msgs::GoalID cancel_msg;
  cancel_msg.stamp = ros::Time(0,0);
  cancel_msg.id = list_handle_.getElem()->getActionGoal()->goal_id.id;

  if (gm_->cancel_func_)
    gm_->cancel_func_(cancel_msg);

  list_handle_.getElem()->transitionToState(*this, CommState::WAITING_FOR_CANCEL_ACK);
}

template<class ActionSpec>
bool ClientGoalHandle<ActionSpec>::operator==(const ClientGoalHandle<ActionSpec>& rhs)
{
  // Check if both are inactive
  if (!active_ && !rhs.active_)
    return true;

  // Check if one or the other is inactive
  if (!active_ || !rhs.active_)
    return false;

  return (list_handle_ == rhs.list_handle_) ;
}

template<class ActionSpec>
bool ClientGoalHandle<ActionSpec>::operator!=(const ClientGoalHandle<ActionSpec>& rhs)
{
  return !(*this==rhs);
}

}
