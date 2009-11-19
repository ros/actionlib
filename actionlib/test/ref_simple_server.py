#!/usr/bin/env python
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

# Author: Alexander Sorokin. 
# Based on code from ref_server.cpp by Vijay Pradeep
PKG='actionlib'
import roslib; roslib.load_manifest(PKG)
import rospy

import sys

from actionlib.simple_action_server import SimpleActionServer
from actionlib.msg import TestAction

class RefSimpleServer:

    def __init__(self,name):
        action_spec=TestAction
        self.srv=SimpleActionServer(name,action_spec,self.execute);

        rospy.loginfo("Creating reference SimpleActionServer [%s]\n", name);

        self.saved_goals=[]

    def execute(self,goal):
        
        rospy.loginfo("Got goal %d", int(goal.goal.goal))
        if goal.goal.goal == 1:
            self.srv.set_succeeded();

        elif goal.goal.goal == 2:
            self.srv.set_aborted();

        elif goal.goal.goal == 3:
            self.srv.set_aborted();


        elif goal.goal.goal == 4:
            self.srv.set_aborted();

        elif goal.goal.goal == 5:
            self.srv.set_aborted();

        elif goal.goal.goal == 6:
            self.srv.set_aborted();

        if goal.goal.goal == 7:
            rospy.sleep(rospy.Duration(1));
            self.srv.set_succeeded();

        if goal.goal.goal == 8:
            """This one succeeds only on non-preempted goals"""
            rospy.sleep(rospy.Duration(1));
            print "preempt?", self.srv.preempt_request
            if self.srv.preempt_request:
                self.srv.set_aborted()
            else:
                self.srv.set_succeeded();

        else:
            pass

        rospy.loginfo("END execute")

    def cancelCallback(self,gh):
        pass

if __name__=="__main__":
  rospy.init_node("ref_simple_server");
  ref_server = RefSimpleServer("reference_simple_action");

  rospy.spin();



