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

class PlusTest(unittest.TestCase):
    def setUp(self):
        self.result = False
        self.value = 1.9
        self.pub = rospy.Publisher('Plus/in', Float64)
        rospy.Subscriber("remapped_output", Float64, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
        if data.data ==  self.value + 2.1:
            self.result = True

    def test_local(self):
        for i in range(1, 10):
            self.pub.publish(Float64(1.9))
            rospy.loginfo("Sent 1.9");
            rospy.sleep(1.0)
        self.assertTrue(self.result)

if __name__ == '__main__':
    rospy.init_node('plus_local')
    rostest.unitrun('test_nodelet', 'test_local', PlusTest, coverage_packages=['nodelet'])
