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

# Author: Stuart Glaser

import sys
import threading
import time
import uuid

PKG = 'bondtest'
import roslib; roslib.load_manifest(PKG)
import rospy
import bondpy
from bondtest.srv import *

import unittest
import rostest

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

TOPIC = "test_bond_topic"
def gen_id():
    return "test_" + str(uuid.uuid4())

class CallbackTests(unittest.TestCase):

    def test_die_in_life_callback(self):
        id = gen_id()
        a = bondpy.Bond(TOPIC, id)
        b = bondpy.Bond(TOPIC, id)

        a.set_formed_callback(a.break_bond)

        a.start()
        b.start()

        self.assertTrue(a.wait_until_formed(rospy.Duration(5.0)))
        self.assertTrue(b.wait_until_broken(rospy.Duration(3.0)))
        del a, b

def main():
    rospy.init_node('test_callbacks_python', anonymous=True, disable_signals=True)
    rostest.run(PKG, 'test_callbacks_python', CallbackTests, sys.argv)

if __name__ == '__main__': main()
