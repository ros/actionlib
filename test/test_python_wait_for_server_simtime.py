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

import unittest
import rospy
from actionlib import SimpleActionClient
from actionlib.msg import TestAction
import time


class TestWaitForServerWithSimTime(unittest.TestCase):
    """
    This test checks the behavior of wait_for_server() 
    in combination with simulated time (use_sim_time).
    """

    def setUp(self):
        self.client = SimpleActionClient('action_server_that_does_not_exist', TestAction)
        use_sim_time_param = rospy.get_param("/use_sim_time", False)
        self.assertTrue(use_sim_time_param, "Expected 'use_sim_time' to be true for this test!")

    def test_wait_without_clock_published(self):
        """
        Checks that wait_for_server()
        """
        start_time = time.time()
        server_online = self.client.wait_for_server(timeout=rospy.Duration(1.0))
        time_delta = time.time() - start_time

        self.assertGreaterEqual(time_delta, 1.0, "wait_for_server() did not wait long enough")
        self.assertFalse(server_online, "wait_for_server() reported server found, but no server was started")


if __name__ == '__main__':
    import rostest
    rospy.init_node('simple_python_client_sim_time_test')
    rostest.rosrun('actionlib', 'test_simple_action_client_python_sim_time', TestWaitForServerWithSimTime)
