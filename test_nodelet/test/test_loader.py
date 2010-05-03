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
import unittest
import rostest
import random

from nodelet.srv import *

class TestLoader(unittest.TestCase):
    def test_loader(self):
        load = rospy.ServiceProxy('/nodelet_manager/load_nodelet', NodeletLoad)
        unload = rospy.ServiceProxy('/nodelet_manager/unload_nodelet', NodeletUnload)
        list = rospy.ServiceProxy('/nodelet_manager/list', NodeletList)
        
        load.wait_for_service()
        unload.wait_for_service()
        list.wait_for_service()
        
        req = NodeletLoadRequest()
        req.name = "/my_nodelet"
        req.type = "test_nodelet/Plus"
        
        res = load.call(req)
        self.assertTrue(res.success)
        
        req = NodeletListRequest()
        res = list.call(req)
        self.assertTrue("/my_nodelet" in res.nodelets)
        
        req = NodeletUnloadRequest()
        req.name = "/my_nodelet"
        res = unload.call(req)
        self.assertTrue(res.success)

if __name__ == '__main__':
    rospy.init_node('test_loader')
    rostest.unitrun('test_loader', 'test_loader', TestLoader)

