#!/usr/bin/env python3
# **********************************************************
#  Software License Agreement (BSD License)
#
#   Copyright (c) 2009, Willow Garage, Inc.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Eitan Marder-Eppstein
# **********************************************************
#  Modifed by Christopher Holley
"""
usage: %prog
"""

PKG = 'actionlib_tools'

import roslib.message
from optparse import OptionParser
import wx
import rospy
import actionlib
import threading
import rostopic
from io import BytesIO
from actionlib_tools.library import to_yaml, yaml_msg_str
from actionlib_tools.dynamic_action import DynamicAction
from actionlib_msgs.msg import GoalStatus
from collections import namedtuple
import inspect
import rospkg
import os

action_type = namedtuple("Action", ["topic", "type"])

class AXClientApp(wx.App):
    def __init__(self, dynamic_actions, action_topics_list):
        self.master_idx = 0 # will be used by code to figure out which state to display
        self.dynamic_actions = dynamic_actions
        self.clients = []
        self.action_topics_list = action_topics_list
        self.results = []
        self.goals = []
        self.feedbacks = []
        self.statuses = []
        self.status_bgs = []
        self.server_statuses = []
        self.server_status_bgs = []
        self.cancel_buttons = [] # bools
        self.raw_message_yamls = [] # raw message files to view comments/enums
        
        self.current_client = None
        self.current_result = None
        self.current_goal = None
        self.current_feedback = None
        self.current_status = None
        self.current_status_bg = None
        self.current_server_status = None
        self.current_server_status_bgs = None
        self.current_cancel_button = None
        self.current_raw_message = None

        self.FIRST_LOOP = True

        self.active_cbs = {} # master idx, function
        self.done_cbs = {} # master idx, function
        self.feedback_cbs = {} # master idx, function   

        
        print(action_topics_list[0])
        print(len(action_topics_list))
        print(len(dynamic_actions))
        for idx, (key, val) in enumerate(dynamic_actions.items()):
            #print(idx, key, val)
            self.clients.append(actionlib.SimpleActionClient(self.action_topics_list[idx].topic, val.action))
            done_cb = self.done_cb_maker()
            active_cb = self.active_cb_maker()
            feedback_cb = self.feedback_cb_maker()

            self.done_cbs[done_cb] = idx
            self.active_cbs[active_cb] = idx
            self.feedback_cbs[feedback_cb] = idx

            # later looked up with getattr
            setattr(self, str(idx)+"_done_cb", done_cb)
            setattr(self, str(idx)+"_feedback_cb", feedback_cb)
            setattr(self, str(idx)+"_active_cb", active_cb)


        print(self.clients)
        assert len(self.clients) == len(self.action_topics_list) == len(dynamic_actions)

        wx.App.__init__(self)
        self.condition = threading.Condition()

    def set_status(self, label, color, idx_override=None):
        if idx_override:
            idx = idx_override
        else:
            idx = self.master_idx
        self.status_bgs[idx] = color
        self.statuses[idx] = label
        self.current_status_bg.SetBackgroundColour(self.status_bgs[self.master_idx])
        self.current_status.SetLabel(self.statuses[self.master_idx])

    def set_cancel_button(self, enabled, idx_override=None):
        if idx_override:
            idx = idx_override
        else:
            idx = self.master_idx
        self.cancel_buttons[idx] = enabled
        if self.cancel_buttons[self.master_idx]:
            self.cancel_goal.Enable()
        else:
            self.cancel_goal.Disable()

    def set_server_status(self, label, color, enabled):
        self.server_status_bgs[self.master_idx] = color
        self.server_statuses[self.master_idx] = label

        self.current_server_status_bg.SetBackgroundColour(color)
        self.current_server_status.SetLabel(label)
        if enabled:
            self.send_goal.Enable()
        else:
            self.send_goal.Disable()
    
    def update_vars(self):
        selection = self.action_selector.GetSelection()
        if selection == wx.NOT_FOUND:
            rospy.loginfo_throttle(3, "No item selected")
            return
        
        idx = self.master_idx
        self.goals[idx] = self.current_goal.GetValue()
        
        if selection != self.master_idx or self.FIRST_LOOP:
            #print("Master idx now = ", selection)
            #print("Goal type=", self.action_topics_list[selection].type)
            idx = selection
            self.master_idx = selection
            #self.print_state()
            self.current_client = self.clients[idx]
            self.current_goal.SetValue(self.goals[idx])
            self.current_raw_message.SetValue(self.raw_message_yamls[idx])
            # force refresh and update to be sure screen gets repainted, not always the case without this
            self.current_raw_message.Refresh()
            self.current_goal.Refresh()
            self.current_raw_message.Update()
            self.current_goal.Update()
            self.action_selector.Refresh()
            self.action_selector.Update()
            self.FIRST_LOOP = False

        self.current_result.SetValue(to_yaml(self.results[idx]))
        self.current_feedback.SetValue(to_yaml(self.feedbacks[idx]))
        self.current_status.SetLabel(self.statuses[idx])
        self.current_status_bg.SetBackgroundColour(self.status_bgs[idx])
        self.current_server_status.SetLabel(self.server_statuses[idx])
        self.current_server_status_bg.SetBackgroundColour(self.server_status_bgs[idx])

    def server_check(self, event):
        #print("Server check")
        self.update_vars()
        TIMEOUT = 0.01
        if self.current_client.wait_for_server(rospy.Duration(TIMEOUT)):
            wx.CallAfter(self.set_server_status, "Connected to server",
                         wx.Colour(192, 252, 253), True)
        else:
            wx.CallAfter(self.set_server_status, "Disconnected from server",
                         wx.Colour(200, 0, 0), False)

    def on_cancel(self, event):
        # we'll cancel the current goal
        self.current_client.cancel_goal()
        self.set_status("Canceling goal", wx.Colour(211, 34, 243))

    def on_goal(self, event):
        idx = self.master_idx
        try:
            dynact = None
            for newidx, (_, val) in enumerate(self.dynamic_actions.items()):
                if newidx == idx:
                    dynact = val
            assert dynact
            self.goal_msg = yaml_msg_str(dynact.goal,
                                         self.current_goal.GetValue())
            buff = BytesIO()
            self.goal_msg.serialize(buff)
            
            # python is actually so cool for this
            # always wanted to call functions with strings
            done_cb = getattr(self, str(self.master_idx) + "_done_cb")
            active_cb = getattr(self, str(self.master_idx) + "_active_cb")
            feedback_cb = getattr(self, str(self.master_idx) + "_feedback_cb")

            assert done_cb and active_cb and feedback_cb
            # send the goal to the action server and register the relevant
            # callbacks
            self.current_client.send_goal(self.goal_msg, done_cb, active_cb, feedback_cb)
            self.set_status("Goal is pending", wx.Colour(255, 174, 59))
            self.set_cancel_button(True)
            print("Goal sent successfully")

        except roslib.message.SerializationError as e:
            self.goal_msg = None
            wx.MessageBox(str(e), "Error serializing goal", wx.OK)

    def set_result(self, result, idx_override=None):
        if idx_override:
            idx = idx_override
        else:
            idx = self.master_idx

        self.results[idx] = result

    def status_gui(self, status):
        return {GoalStatus.PENDING: ['PENDING', wx.Colour(255, 174, 59)],
                GoalStatus.ACTIVE: ['ACTIVE', wx.Colour(0, 255, 0)],
                GoalStatus.PREEMPTED: ['PREEMPTED', wx.Colour(255, 252, 16)],
                GoalStatus.SUCCEEDED: ['SUCCEEDED', wx.Colour(38, 250, 253)],
                GoalStatus.ABORTED: ['ABORTED', wx.Colour(200, 0, 0)],
                GoalStatus.REJECTED: ['REJECTED', wx.Colour(253, 38, 159)],
                GoalStatus.PREEMPTING: ['PREEMPTING', wx.Colour(253, 38, 159)],
                GoalStatus.RECALLING: ['RECALLING', wx.Colour(230, 38, 253)],
                GoalStatus.RECALLED: ['RECALLED', wx.Colour(230, 38, 253)],
                GoalStatus.LOST: ['LOST', wx.Colour(255, 0, 0)]}[status]
    
    # really "cool" or horrible hack depending on who you ask.
    # problem is that if the user clicks to another action after sending a goal, then we have to be able to have the feedback update the correct state
    # so we need the master_idx, but that will have changed.
    # way to do this while also not rewriting actionlib code it to make unqiue cbs for each master_idx and then look up what the master idx is based on the function
    
    def done_cb_maker(self):
        def done_cb(state, result):
            # lookup based on what function we have
            previous_idx = int(self.done_cbs[done_cb])
            status_string, status_color = self.status_gui(state)
            wx.CallAfter(self.set_status, ''.join(["Goal finished with status: ",
                                                status_string]), status_color, idx_override=previous_idx)
            wx.CallAfter(self.set_result, result, idx_override=previous_idx)
            wx.CallAfter(self.set_cancel_button, False, idx_override=previous_idx)
        return done_cb

    def active_cb_maker(self):
        def active_cb():
            wx.CallAfter(self.set_status, "Goal is active", wx.Colour(0, 200, 0))
        return active_cb

    def feedback_cb_maker(self):        
        def feedback_cb(feedback):
            i = int(self.feedback_cbs[feedback_cb])
            wx.CallAfter(self.set_feedback, feedback, idx_override=i)
        return feedback_cb

    def set_feedback(self, feedback, idx_override=None):
        if idx_override:
            idx = idx_override
        else:
            idx = self.master_idx
        
        self.feedbacks[idx] = feedback

        #try:
        #    self.current_feedback.SetValue(to_yaml(self.feedbacks[idx]))
        #except UnicodeDecodeError:
        #    self.current_feedback.SetValue("Cannot display feedback due to unprintable characters")


    def OnQuit(self):
        self.server_check_timer.Stop()

    def OnInit(self):
        print("Running init")

        self.frame = wx.Frame(
            None, -1,
            'General Actionlib Client'
        )

        self.sz = wx.BoxSizer(wx.VERTICAL)

        self.action_selector = wx.ListBox(self.frame, -1, style=wx.TE_MULTILINE)
        for idx, topic in enumerate(self.action_topics_list):
            print(topic, idx)
            self.action_selector.Insert(topic[0], idx)

        self.action_selector_st_bx = wx.StaticBox(self.frame, -1, "Action Selector")
        self.action_selector_st = wx.StaticBoxSizer(self.action_selector_st_bx, wx.VERTICAL)
        self.action_selector_st.Add(self.action_selector, 1, wx.EXPAND)

        for idx, (_, val) in enumerate(self.dynamic_actions.items()):
            self.goals.append(None)
            self.results.append(None)
            self.feedbacks.append(None)
            self.results.append(None)
            self.statuses.append(None)
            self.status_bgs.append(None)
            self.server_statuses.append(None)
            self.server_status_bgs.append(None)
            self.cancel_buttons.append(None)
            self.raw_message_yamls.append(None)

        rospack = rospkg.RosPack()
        for idx, (_, val) in enumerate(self.dynamic_actions.items()):
            self.goals[idx] = to_yaml(val.goal())
            self.results[idx] = ""
            self.feedbacks[idx] = ""
            self.results[idx] = ""
            self.statuses[idx] = "No Goal"
            self.status_bgs[idx] = wx.Colour(200, 0, 0)
            self.server_statuses[idx] = "Disconnected from server :("
            self.server_status_bgs[idx] = wx.Colour(200, 0, 0)
            self.cancel_buttons[idx] = False
            
            msg_dir = os.path.join(rospack.get_path(self.action_topics_list[idx].type.split("/")[0]), 'action/')
            msg_dir = msg_dir + self.action_topics_list[idx].type.split("/")[1].replace("Action", ".action")
            with open(msg_dir, "r") as f:
                data = f.read()
                self.raw_message_yamls[idx] = data
            
        self.current_goal = wx.TextCtrl(self.frame, -1, style=wx.TE_MULTILINE)
        self.current_goal.SetValue(self.goals[self.master_idx])
        self.goal_st_bx = wx.StaticBox(self.frame, -1, "Goal")
        self.goal_st = wx.StaticBoxSizer(self.goal_st_bx, wx.VERTICAL)
        self.goal_st.Add(self.current_goal, 1, wx.EXPAND)
        self.current_goal.AutoComplete

        self.current_feedback = wx.TextCtrl(self.frame, -1, style=(wx.TE_MULTILINE |
                                                           wx.TE_READONLY))
        self.feedback_st_bx = wx.StaticBox(self.frame, -1, "Feedback")
        self.feedback_st = wx.StaticBoxSizer(self.feedback_st_bx, wx.VERTICAL)
        self.feedback_st.Add(self.current_feedback, 1, wx.EXPAND)


        self.current_raw_message = wx.TextCtrl(self.frame, -1, style=(wx.TE_MULTILINE |
                                                         wx.TE_READONLY))
        self.raw_message_st_bx = wx.StaticBox(self.frame, -1, "Raw Message")
        self.raw_message_st = wx.StaticBoxSizer(self.raw_message_st_bx, wx.HORIZONTAL)
        self.raw_message_st.Add(self.current_raw_message, 1, wx.EXPAND)

        self.current_result = wx.TextCtrl(self.frame, -1, style=(wx.TE_MULTILINE |
                                                         wx.TE_READONLY))
        self.result_st_bx = wx.StaticBox(self.frame, -1, "Result")
        self.result_st = wx.StaticBoxSizer(self.result_st_bx, wx.VERTICAL)
        self.result_st.Add(self.current_result, 1, wx.EXPAND)

        self.send_goal = wx.Button(self.frame, -1, label="SEND GOAL")
        self.send_goal.Bind(wx.EVT_BUTTON, self.on_goal)
        self.send_goal.Disable()

        self.cancel_goal = wx.Button(self.frame, -1, label="CANCEL GOAL")
        self.cancel_goal.Bind(wx.EVT_BUTTON, self.on_cancel)
        self.cancel_goal.Disable()

        self.current_status_bg = wx.Panel(self.frame, -1)
        self.current_status_bg.SetBackgroundColour(wx.Colour(200, 0, 0))
        self.current_status = wx.StaticText(self.current_status_bg, -1, label="No Goal")

        self.current_server_status_bg = wx.Panel(self.frame, -1)
        self.current_server_status_bg.SetBackgroundColour(wx.Colour(200, 0, 0))
        self.current_server_status = wx.StaticText(self.current_server_status_bg, -1, label="Disconnected from server.")

        self.sz.Add(self.action_selector_st, 1, wx.EXPAND)
        self.sz.Add(self.raw_message_st, 2, wx.EXPAND)
        self.sz.Add(self.goal_st, 2, wx.EXPAND)
        self.sz.Add(self.feedback_st, 1, wx.EXPAND)
        self.sz.Add(self.result_st, 1, wx.EXPAND)
        self.sz.Add(self.send_goal, 0, wx.EXPAND)
        self.sz.Add(self.cancel_goal, 0, wx.EXPAND)
        self.sz.Add(self.current_status_bg, 0, wx.EXPAND)
        self.sz.Add(self.current_server_status_bg, 0, wx.EXPAND)

        self.frame.SetSizer(self.sz)

        self.server_check_timer = wx.Timer(self.frame)
        self.frame.Bind(wx.EVT_TIMER, self.server_check,
                        self.server_check_timer)
        self.server_check_timer.Start(100)

        self.sz.Layout()
        
        self.frame.Show()

        return True

def main():
    rospy.init_node('axclient', anonymous=True)
    
    topics = rospy.get_published_topics()
    action_topics = []

    for topic in topics: 
        # every action should have a result topic, could do the same thing with /goal or /feedback
        if "ActionResult" in topic[1]:
            topic[0] = topic[0].replace("/result", "") 
            topic[1] = topic[1].replace("Result", "")
            #print(topic)
            action_topics.append(action_type(topic[0], topic[1]))

    dynactions = {}
    for action in action_topics:
        #print(action.type)
        dynactions[action.topic] = DynamicAction(action.type)

    app = AXClientApp(dynactions, action_topics)
    app.MainLoop()
    app.OnQuit()
    rospy.signal_shutdown('GUI shutdown')

if __name__ == '__main__':
    main()
