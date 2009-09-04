#! /usr/bin/python
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

# Author: Stuart Glaser

import sys
import cStringIO
import re
import roslib, roslib.msgs
import os, os.path

IODELIM   = '---'
COMMENTCHAR = roslib.msgs.COMMENTCHAR

class ActionSpecException(Exception): pass

def parse_action_spec(text, package_context = ''):
    pieces = [cStringIO.StringIO()]
    for l in text.split('\n'):
        if l.startswith(IODELIM):
            pieces.append(cStringIO.StringIO())
        else:
            pieces[-1].write(l + '\n')
    return [p.getvalue() for p in pieces]

def write_file(filename, text):
    f = open(filename, 'w')
    f.write(text)
    f.close()

def main():

    if len(sys.argv) < 2:
        print "Need to give a package path"
        sys.exit(1)
    pkg = os.path.abspath(sys.argv[1])

    if not os.path.exists(os.path.join(pkg, 'manifest.xml')):
        print "Not a package %s" % pkg
        sys.exit(1)
    
    output_dir = os.path.join(pkg, 'msg')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    action_dir = os.path.join(pkg, 'action')
    for action_file in os.listdir(action_dir):
        if action_file.endswith('.action'):
            filename = os.path.join(action_dir, action_file)

            f = open(filename)
            action_spec = f.read()
            f.close()

            name = os.path.basename(filename)[:-7]
            print "Generating for action %s" % name

            pieces = parse_action_spec(action_spec)
            if len(pieces) != 3:
                raise ActionSpecException("%s: wrong number of pieces, %d"%(filename,len(pieces)))
            goal, result, feedback = pieces

            action_msg = """
%sActionGoal action_goal
%sActionResult action_result
%sActionFeedback action_feedback
""" % (name, name, name)

            goal_msg = goal
            action_goal_msg = """
Header header
actionlib_msgs/GoalID goal_id
%sGoal goal
""" % name

            result_msg = result
            action_result_msg = """
Header header
actionlib_msgs/GoalStatus status
%sResult result
""" % name

            feedback_msg = feedback
            action_feedback_msg = """
Header header
actionlib_msgs/GoalStatus status
%sFeedback feedback
""" % name

            write_file(os.path.join(output_dir, "%sAction.msg"%name), action_msg)
            write_file(os.path.join(output_dir, "%sGoal.msg"%name), goal_msg)
            write_file(os.path.join(output_dir, "%sActionGoal.msg"%name), action_goal_msg)
            write_file(os.path.join(output_dir, "%sResult.msg"%name), result_msg)
            write_file(os.path.join(output_dir, "%sActionResult.msg"%name), action_result_msg)
            write_file(os.path.join(output_dir, "%sFeedback.msg"%name), feedback_msg)
            write_file(os.path.join(output_dir, "%sActionFeedback.msg"%name), action_feedback_msg)


if __name__ == '__main__': main()
