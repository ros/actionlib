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


import roslib; roslib.load_manifest('test_nodelet')
import rospy
from std_msgs.msg import Float64
import unittest
import rostest
import random

class PlusTester:
    def __init__(self, topic_in, topic_out, delta):
        self.pub = rospy.Publisher(topic_in, Float64)
        self.sub = rospy.Subscriber(topic_out, Float64, self.callback)
        self.expected_delta = delta
        self.send_value = random.random()
        self.recieved = False
        self.result = False
        self.delta = delta

    def run(self, cycles = 10):
        for i in range(1, cycles):
            self.pub.publish(Float64(self.send_value))
            rospy.loginfo("Sent %f"%self.send_value);
            if self.recieved == True:
                break
            rospy.sleep(1.0)
        return self.result 

    def callback(self, data):
        rospy.loginfo(rospy.get_name()+" I heard %s which was a change of %f",data.data, data.data-self.send_value)
        if data.data ==  self.send_value + self.delta:
            self.result = True
        self.recieved = True

class TestPlus(unittest.TestCase):
    def test_param(self):
        pb = PlusTester("Plus/in", "Plus/out", 2.1)
        self.assertTrue(pb.run())

    def test_default_param(self):
        pb = PlusTester("Plus2/in", "Plus2/out", 10.0)
        self.assertTrue(pb.run())

    def test_standalone(self):
        pb = PlusTester("Plus2/out", "Plus3/out", 2.5)
        self.assertTrue(pb.run())

    def test_remap(self):
        pb = PlusTester("plus_remap/in", "remapped_output", 2.1)
        self.assertTrue(pb.run())

    def test_chain(self):
        pb = PlusTester("Plus2/in", "Plus3/out", 12.5)
        self.assertTrue(pb.run())


if __name__ == '__main__':
    rospy.init_node('plus_local')
    rostest.unitrun('test_nodelet', 'test_local', TestPlus, coverage_packages=['nodelet'])

