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
#ifndef ACTIONLIB_ACTION_SERVER_IMP_H_
#define ACTIONLIB_ACTION_SERVER_IMP_H_
namespace actionlib {
  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      bool auto_start)
    : node_(n, name), goal_callback_(boost::function<void (GoalHandle)>()),
      cancel_callback_(boost::function<void (GoalHandle)>()), started_(auto_start), guard_(new DestructionGuard){
    //if we're to autostart... then we'll initialize things
    if(started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name)
    : node_(n, name), goal_callback_(boost::function<void (GoalHandle)>()),
      cancel_callback_(boost::function<void (GoalHandle)>()), started_(true), guard_(new DestructionGuard){
    //if we're to autostart... then we'll initialize things
    if(started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      boost::function<void (GoalHandle)> goal_cb,
      boost::function<void (GoalHandle)> cancel_cb,
      bool auto_start)
    : node_(n, name), goal_callback_(goal_cb), cancel_callback_(cancel_cb), started_(auto_start), guard_(new DestructionGuard) {
    //if we're to autostart... then we'll initialize things
    if(started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      boost::function<void (GoalHandle)> goal_cb,
      boost::function<void (GoalHandle)> cancel_cb)
    : node_(n, name), goal_callback_(goal_cb), cancel_callback_(cancel_cb), started_(true), guard_(new DestructionGuard) {
    //if we're to autostart... then we'll initialize things
    if(started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      boost::function<void (GoalHandle)> goal_cb,
      bool auto_start)
    : node_(n, name), goal_callback_(goal_cb), cancel_callback_(boost::function<void (GoalHandle)>()), started_(auto_start), guard_(new DestructionGuard) {
    //if we're to autostart... then we'll initialize things
    if(started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::~ActionServer(){
    //block until we can safely destruct
    guard_->destruct();
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::initialize(){
    result_pub_ = node_.advertise<ActionResult>("result", 50);
    feedback_pub_ = node_.advertise<ActionFeedback>("feedback", 50);
    status_pub_ = node_.advertise<actionlib_msgs::GoalStatusArray>("status", 50, true);

    //read the frequency with which to publish status from the parameter server
    //if not specified locally explicitly, use search param to find actionlib_status_frequency
    double status_frequency, status_list_timeout;
    if(!node_.getParam("status_frequency", status_frequency))
    {
      std::string status_frequency_param_name;
      if(!node_.searchParam("actionlib_status_frequency", status_frequency_param_name))
        status_frequency = 5.0;
      else
        node_.param(status_frequency_param_name, status_frequency, 5.0);
    }
    else
      ROS_WARN_NAMED("actionlib", "You're using the deprecated status_frequency parameter, please switch to actionlib_status_frequency.");

    node_.param("status_list_timeout", status_list_timeout, 5.0);

    status_list_timeout_ = ros::Duration(status_list_timeout);

    if(status_frequency > 0){
      status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
          boost::bind(&ActionServer::publishStatus, this, _1));
    }

    goal_sub_ = node_.subscribe<ActionGoal>("goal", 50,
        boost::bind(&ActionServer::goalCallback, this, _1));

    cancel_sub_ = node_.subscribe<actionlib_msgs::GoalID>("cancel", 50,
        boost::bind(&ActionServer::cancelCallback, this, _1));

  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::registerGoalCallback(boost::function<void (GoalHandle)> cb){
    goal_callback_ = cb;
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::registerCancelCallback(boost::function<void (GoalHandle)> cb){
    cancel_callback_ = cb;
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishResult(const actionlib_msgs::GoalStatus& status, const Result& result){
    boost::recursive_mutex::scoped_lock lock(lock_);
    //we'll create a shared_ptr to pass to ROS to limit copying
    boost::shared_ptr<ActionResult> ar(new ActionResult);
    ar->header.stamp = ros::Time::now();
    ar->status = status;
    ar->result = result;
    ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());
    result_pub_.publish(ar);
    publishStatus();
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishFeedback(const actionlib_msgs::GoalStatus& status, const Feedback& feedback){
    boost::recursive_mutex::scoped_lock lock(lock_);
    //we'll create a shared_ptr to pass to ROS to limit copying
    boost::shared_ptr<ActionFeedback> af(new ActionFeedback);
    af->header.stamp = ros::Time::now();
    af->status = status;
    af->feedback = feedback;
    ROS_DEBUG_NAMED("actionlib", "Publishing feedback for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());
    feedback_pub_.publish(af);
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::cancelCallback(const boost::shared_ptr<const actionlib_msgs::GoalID>& goal_id){
    boost::recursive_mutex::scoped_lock lock(lock_);

    //if we're not started... then we're not actually going to do anything
    if(!started_)
      return;

    //we need to handle a cancel for the user
    ROS_DEBUG_NAMED("actionlib", "The action server has received a new cancel request");
    bool goal_id_found = false;
    for(typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
      //check if the goal id is zero or if it is equal to the goal id of
      //the iterator or if the time of the iterator warrants a cancel
      if(
          (goal_id->id == "" && goal_id->stamp == ros::Time()) //id and stamp 0 --> cancel everything
          || goal_id->id == (*it).status_.goal_id.id //ids match... cancel that goal
          || (goal_id->stamp != ros::Time() && (*it).status_.goal_id.stamp <= goal_id->stamp) //stamp != 0 --> cancel everything before stamp
        ){
        //we need to check if we need to store this cancel request for later
        if(goal_id->id == (*it).status_.goal_id.id)
          goal_id_found = true;

        //attempt to get the handle_tracker for the list item if it exists
        boost::shared_ptr<void> handle_tracker = (*it).handle_tracker_.lock();

        if((*it).handle_tracker_.expired()){
          //if the handle tracker is expired, then we need to create a new one
          HandleTrackerDeleter<ActionSpec> d(this, it, guard_);
          handle_tracker = boost::shared_ptr<void>((void *)NULL, d);
          (*it).handle_tracker_ = handle_tracker;

          //we also need to reset the time that the status is supposed to be removed from the list
          (*it).handle_destruction_time_ = ros::Time();
        }

        //set the status of the goal to PREEMPTING or RECALLING as approriate
        //and check if the request should be passed on to the user
        GoalHandle gh(it, this, handle_tracker, guard_);
        if(gh.setCancelRequested()){
          //make sure that we're unlocked before we call the users callback
          lock_.unlock();

          //call the user's cancel callback on the relevant goal
          cancel_callback_(gh);

          //lock for further modification of the status list
          lock_.lock();
        }
      }
    }

    //if the requested goal_id was not found, and it is non-zero, then we need to store the cancel request
    if(goal_id->id != "" && !goal_id_found){
      typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.insert(status_list_.end(),
          StatusTracker<ActionSpec> (*goal_id, actionlib_msgs::GoalStatus::RECALLING));
      //start the timer for how long the status will live in the list without a goal handle to it
      (*it).handle_destruction_time_ = ros::Time::now();
    }

    //make sure to set last_cancel_ based on the stamp associated with this cancel request
    if(goal_id->stamp > last_cancel_)
      last_cancel_ = goal_id->stamp;
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::goalCallback(const boost::shared_ptr<const ActionGoal>& goal){
    boost::recursive_mutex::scoped_lock lock(lock_);

    //if we're not started... then we're not actually going to do anything
    if(!started_)
      return;

    ROS_DEBUG_NAMED("actionlib", "The action server has received a new goal request");

    //we need to check if this goal already lives in the status list
    for(typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
      if(goal->goal_id.id == (*it).status_.goal_id.id){

        // The goal could already be in a recalling state if a cancel came in before the goal
        if ( (*it).status_.status == actionlib_msgs::GoalStatus::RECALLING ) {
          (*it).status_.status = actionlib_msgs::GoalStatus::RECALLED;
          publishResult((*it).status_, Result());
        }

        //if this is a request for a goal that has no active handles left,
        //we'll bump how long it stays in the list
        if((*it).handle_tracker_.expired()){
          (*it).handle_destruction_time_ = ros::Time::now();
        }

        //make sure not to call any user callbacks or add duplicate status onto the list
        return;
      }
    }

    //if the goal is not in our list, we need to create a StatusTracker associated with this goal and push it on
    typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.insert(status_list_.end(), StatusTracker<ActionSpec> (goal));

    //we need to create a handle tracker for the incoming goal and update the StatusTracker
    HandleTrackerDeleter<ActionSpec> d(this, it, guard_);
    boost::shared_ptr<void> handle_tracker((void *)NULL, d);
    (*it).handle_tracker_ = handle_tracker;

    //check if this goal has already been canceled based on its timestamp
    if(goal->goal_id.stamp != ros::Time() && goal->goal_id.stamp <= last_cancel_){
      //if it has... just create a GoalHandle for it and setCanceled
      GoalHandle gh(it, this, handle_tracker, guard_);
      gh.setCanceled(Result(), "This goal handle was canceled by the action server because its timestamp is before the timestamp of the last cancel request");
    }
    else{
      GoalHandle gh = GoalHandle(it, this, handle_tracker, guard_);

      //make sure that we unlock before calling the users callback
      lock_.unlock();

      //now, we need to create a goal handle and call the user's callback
      goal_callback_(gh);

      lock_.lock();
    }
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::start(){
    initialize();
    started_ = true;
    publishStatus();
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishStatus(const ros::TimerEvent& e){
    boost::recursive_mutex::scoped_lock lock(lock_);
    //we won't publish status unless we've been started
    if(!started_)
      return;

    publishStatus();
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishStatus(){
    boost::recursive_mutex::scoped_lock lock(lock_);
    //build a status array
    actionlib_msgs::GoalStatusArray status_array;

    status_array.header.stamp = ros::Time::now();

    status_array.status_list.resize(status_list_.size());

    unsigned int i = 0;
    for(typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.begin(); it != status_list_.end();){
      status_array.status_list[i] = (*it).status_;

      //check if the item is due for deletion from the status list
      if((*it).handle_destruction_time_ != ros::Time()
          && (*it).handle_destruction_time_ + status_list_timeout_ < ros::Time::now()){
        it = status_list_.erase(it);
      }
      else
        ++it;

      ++i;
    }

    status_pub_.publish(status_array);
  }
};
#endif
