#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Alexander Sorokin. 
# Based on C++ action_server.h by Eitan Marder-Eppstein
from __future__ import with_statement

import roslib; roslib.load_manifest('actionlib')
import rospy

import threading

from actionlib_msgs.msg import *

from goal_id_generator import GoalIDGenerator
from status_tracker import StatusTracker

from handle_tracker_deleter import HandleTrackerDeleter
from server_goal_handle import ServerGoalHandle


def nop_cb(goal_handle):
    pass

def ros_timer(callable,frequency):
    start = rospy.get_time()
    cycle_time = 1.0 / frequency;

    rospy.logdebug("Starting timer");
    while not rospy.is_shutdown():
        start = rospy.get_time()
        callable()

        end = rospy.get_time()
        sleep_time = cycle_time - (end - start)
        if sleep_time > 0:
            rospy.sleep(sleep_time)
        else:
            rospy.logwarn("Timer missed cycle time of %.2f seconds" % (-sleep_time))


class ActionServer:
    """
    * @class ActionServer
    * @brief The ActionServer is a helpful tool for managing goal requests to a
    * node. It allows the user to specify callbacks that are invoked when goal
    * or cancel requests come over the wire, and passes back GoalHandles that
    * can be used to track the state of a given goal request. The ActionServer
    * makes no assumptions about the policy used to service these goals, and
    * sends status for each goal over the wire until the last GoalHandle
    * associated with a goal request is destroyed.
    """
    def __init__(self, ns,  ActionSpec, goal_cb, cancel_cb = nop_cb, auto_start = True):
        """
        * @brief  Constructor for an ActionServer
        * @param  ns/name A namespace for the action server
        * @param  actionspec An explicit specification of the action
        * @param  goal_cb A goal callback to be called when the ActionServer receives a new goal over the wire
        * @param  cancel_cb A cancel callback to be called when the ActionServer receives a new cancel request over the wire
        * @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up
        """
        self.ns=ns;

        try:
            a = ActionSpec()
            
            self.ActionSpec = ActionSpec
            self.ActionGoal = type(a.action_goal)
            self.ActionResult = type(a.action_result)
            self.ActionResultType = type(a.action_result.result)
            self.ActionFeedback = type(a.action_feedback)
        except AttributeError:
            raise ActionException("Type is not an action spec: %s" % str(ActionSpec))


        self.goal_sub = None
        self.cancel_sub = None
        self.status_pub = None
        self.result_pub = None
        self.feedback_pub = None
        
        self.lock = threading.RLock()
        
        self.status_timer = None;

        self.status_list = [];
          
        self.last_cancel = rospy.Time();
        self.status_list_timeout = rospy.Duration();
        
        self.id_generator = GoalIDGenerator();
        
        self.goal_callback = goal_cb;
        assert(self.goal_callback);

        self.cancel_callback = cancel_cb;
        self.auto_start = auto_start; 

        self.started = False;
        
        if self.auto_start:
            self.initialize()
            self.started = True;
            self.publish_status()


    def register_goal_callback(self, cb):
        """ @brief  Register a callback to be invoked when a new goal is received, this will replace any  previously registered callback
        @param  cb The callback to invoke
        """
        self.goal_callback = cb

    def register_cancel_callback(self,cancel_cb):
          """ @brief  Register a callback to be invoked when a new cancel is received, this will replace any  previously registered callback
              @param  cb The callback to invoke
              """
          self.cancel_callback = cancel_cb

    def start(self):
        """
        @brief  Explicitly start the action server, used it auto_start is set to false
        """
        self.initialize();
        self.started = True;
        self.publish_status();


    def initialize(self):
          """
          @brief  Initialize all ROS connections and setup timers
          """
          self.status_pub = rospy.Publisher(self.ns+"/status", GoalStatusArray);
          self.result_pub = rospy.Publisher(self.ns+"/result", self.ActionResult);
          self.feedback_pub = rospy.Publisher(self.ns+"/feedback", self.ActionFeedback);

          self.goal_sub = rospy.Subscriber(self.ns+"/goal", self.ActionGoal,self.internal_goal_callback);
          
          self.cancel_sub = rospy.Subscriber(self.ns+"/cancel", GoalID,self.internal_cancel_callback);

          #read the frequency with which to publish status from the parameter server
          self.status_frequency = rospy.get_param(self.ns+"/status_frequency", 5.0);

          status_list_timeout = rospy.get_param(self.ns+"/status_list_timeout", 5.0);
          self.status_list_timeout = rospy.Duration(status_list_timeout);


          self.status_timer = threading.Thread( None, ros_timer, None, (self.publish_status_async,self.status_frequency) );
          self.status_timer.start();

    def publish_result(self, status, result):
        """
        # @brief  Publishes a result for a given goal
        # @param status The status of the goal with which the result is associated
        # @param result The result to publish
        """

        with self.lock:
            ar = self.ActionResult();
            ar.status = status;
            ar.result = result;
            self.result_pub.publish(ar);          


    def publish_feedback(self, status, feedback):
          """
          * @brief  Publishes feedback for a given goal
          * @param status The status of the goal with which the feedback is associated
          * @param feedback The feedback to publish
          """

          with self.lock:

              af=ActionFeedback();
              af.status = status;
              af.feedback = feedback;
              self.feedback_pub.publish(af);


    def internal_cancel_callback(self, goal_id):
          """
          * @brief  The ROS callback for cancel requests coming into the ActionServer
          """

          with self.lock:

              #if we're not started... then we're not actually going to do anything
              if not self.started:
                  return;

              #we need to handle a cancel for the user
              rospy.logdebug("The action server has received a new cancel request");

              goal_id_found = False;
              for i, it in enumerate(self.status_list):
                  #check if the goal id is zero or if it is equal to the goal id of
                  #the iterator or if the time of the iterator warrants a cancel

                  cancel_everything = (goal_id.id == "" and goal_id.stamp == rospy.Time() )   #rospy::Time()) #id and stamp 0 --> cancel everything
                  cancel_this_one = ( goal_id.id == it.status.goal_id.id)   #ids match... cancel that goal
                  cancel_before_stamp = (goal_id.stamp != rospy.Time() and it.status.goal_id.stamp <= goal_id.stamp)  #//stamp != 0 --> cancel everything before stamp
                  
                  if cancel_everything or cancel_this_one or cancel_before_stamp:
                      #we need to check if we need to store this cancel request for later
                      if goal_id.id == it.status.goal_id.id:
                          goal_id_found = True;

                      #attempt to get the handle_tracker for the list item if it exists
                      handle_tracker = it.handle_tracker;

                      if handle_tracker is None:
                          #if the handle tracker is expired, then we need to create a new one
                          handle_tracker = HandleTrackerDeleter(self, it);
                          it.handle_tracker = handle_tracker;
                      
                          #we also need to reset the time that the status is supposed to be removed from the list
                          it.handle_destruction_time = rospy.Time.now()
                      

                      #set the status of the goal to PREEMPTING or RECALLING as approriate
                      #and check if the request should be passed on to the user
                      gh = ServerGoalHandle(it, self, handle_tracker);
                      if gh.set_cancel_requested():
                          #call the user's cancel callback on the relevant goal
                          self.cancel_callback(gh);


              #if the requested goal_id was not found, and it is non-zero, then we need to store the cancel request
              if goal_id.id != "" and not goal_id_found:
                  tracker= StatusTracker(goal_id, actionlib_msgs.msg.GoalStatus.RECALLING);
                  self.status_list.append(tracker)
                  #start the timer for how long the status will live in the list without a goal handle to it
                  tracker.handle_destruction_time = rospy.Time.now()

              #make sure to set last_cancel_ based on the stamp associated with this cancel request
              if goal_id.stamp > self.last_cancel:
                  self.last_cancel = goal_id.stamp;
              

    def internal_goal_callback(self, goal):
          """
          * @brief  The ROS callback for goals coming into the ActionServer
          """

          with self.lock:
              #if we're not started... then we're not actually going to do anything
              if not self.started:
                  return;

              rospy.logdebug("The action server has received a new goal request");

              #we need to check if this goal already lives in the status list
              for st in self.status_list:
                  if goal.goal_id.id == st.status.goal_id.id:
                      #if this is a request for a goal that has no active handles left,
                      #we'll bump how long it stays in the list
                      if st.handle_tracker is None:
                          st.handle_destruction_time = rospy.Time.now()
                      
                      #make sure not to call any user callbacks or add duplicate status onto the list
                      return;

              #if the goal is not in our list, we need to create a StatusTracker associated with this goal and push it on
              st = StatusTracker(None,None,goal)
              self.status_list.append(st);

              #we need to create a handle tracker for the incoming goal and update the StatusTracker
              handle_tracker = HandleTrackerDeleter(self, st);

              st.handle_tracker = handle_tracker;

              #check if this goal has already been canceled based on its timestamp
              gh= ServerGoalHandle(st, self, handle_tracker);
              if goal.goal_id.stamp != rospy.Time() and goal.goal_id.stamp <= self.last_cancel:
                  #if it has... just create a GoalHandle for it and setCanceled
                  gh.set_canceled();
              else:
                  #now, we need to create a goal handle and call the user's callback
                  self.goal_callback(gh);

        
    def publish_status_async(self):
          """
          * @brief  Publish status for all goals on a timer event
          """
          rospy.logdebug("Status async");
          with self.lock:
              #we won't publish status unless we've been started
              if not self.started:
                  return
              self.publish_status();

              

    def publish_status(self):
          """
          * @brief  Explicitly publish status
          """
          with self.lock:
              #build a status array
              status_array = actionlib_msgs.msg.GoalStatusArray()

              #status_array.set_status_list_size(len(self.status_list));

              for i,st in enumerate(self.status_list):
                  #check if the item is due for deletion from the status list
                  if st.handle_destruction_time != rospy.Time() and st.handle_destruction_time + self.status_list_timeout < rospy.Time.now():
                      #del self.status_list[i]
                      pass
                  else:
                      status_array.status_list.append(st.status);


              self.status_pub.publish(status_array)


