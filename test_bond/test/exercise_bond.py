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

PKG = 'test_bond'
import roslib; roslib.load_manifest(PKG)
import rospy
import bondpy
from test_bond.srv import *

import unittest
import rostest

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

def gen_id():
    return "test_" + str(uuid.uuid4())
TOPIC = "test_bond_topic"

test_bond = rospy.ServiceProxy('test_bond', TestBond)

class Exerciser(unittest.TestCase):

    # Bond formed, bond broken remotely
    def test_normal(self):
        id = gen_id()
        test_bond.wait_for_service()
        test_bond(id = id, topic = TOPIC, delay_death = rospy.Duration(2.0))
        rospy.logerr("test_normal: test_bond service call just returned")
        bond = bondpy.Bond(TOPIC, id)
        bond.start()
        
        bond_start_time = time.time()

        formed = bond.wait_until_formed(rospy.Duration(2.0))
        if not formed:
            s = ''
            s += "BONDFAIL\n"
            s += "wait_until_formed returned %s\n" % formed
            s += "Bond details:\n"
            s += "  Started = %s\n" % bond._Bond__started
            s += "  sister_instance_id = %s\n" % bond.sister_instance_id
            s += "  is_shutdown = %s\n" % bond.is_shutdown
            s += "  num connections = %d\n" % bond.pub.get_num_connections()

            formed_later = bond.wait_until_formed(rospy.Duration(20.0))
            s += "Formed later: %s after %.3f seconds\n" % (formed_later, time.time() - bond_start_time)

            rospy.logerr(s)
            self.fail(s)

        #self.assertTrue(bond.wait_until_formed(rospy.Duration(2.0)))
        self.assertTrue(bond.wait_until_broken(rospy.Duration(5.0)))

    # Remote never connects
    def test_no_connect(self):
        id = gen_id()
        # Don't start the other side of the bond
        bond = bondpy.Bond(TOPIC, id)
        bond.start()
        self.assertFalse(bond.wait_until_formed(rospy.Duration(1.0)))
        self.assertTrue(bond.wait_until_broken(rospy.Duration(20.0)))

    # Remote dies (but doesn't report it)
    def test_heartbeat_timeout(self):
        id = gen_id()
        test_bond.wait_for_service()
        test_bond(id=id, topic=TOPIC, delay_death=rospy.Duration(2.0), inhibit_death_message=True)
        bond = bondpy.Bond(TOPIC, id)
        bond.start()

        self.assertTrue(bond.wait_until_formed(rospy.Duration(2.0)))
        self.assertTrue(bond.wait_until_broken(rospy.Duration(10.0)))

    # Local dies, remote acks normally
    def test_i_die(self):
        id = gen_id()
        test_bond.wait_for_service()
        test_bond(id = id, topic = TOPIC, delay_death = rospy.Duration(-1))
        bond = bondpy.Bond(TOPIC, id)
        bond.start()

        self.assertTrue(bond.wait_until_formed(rospy.Duration(2.0)))

        bond.break_bond()
        self.assertTrue(bond.wait_until_broken(rospy.Duration(2.0)))

    # Local dies, remote goes silent
    def test_die_no_ack(self):
        id = gen_id()
        test_bond.wait_for_service()
        test_bond(id=id, topic=TOPIC, delay_death=rospy.Duration(-1), inhibit_death_message=True)
        bond = bondpy.Bond(TOPIC, id)
        bond.start()

        self.assertTrue(bond.wait_until_formed(rospy.Duration(2.0)))

        bond.break_bond()
        self.assertTrue(bond.wait_until_broken(rospy.Duration(5.0)))

    # Local dies, remote stays alive
    def test_die_ignore_death(self):
        id = gen_id()
        test_bond.wait_for_service()
        test_bond(id=id, topic=TOPIC, delay_death=rospy.Duration(-1), inhibit_death=True)
        bond = bondpy.Bond(TOPIC, id)
        bond.start()

        self.assertTrue(bond.wait_until_formed(rospy.Duration(2.0)))

        bond.break_bond()
        self.assertFalse(bond.wait_until_broken(rospy.Duration(1.0)))
        self.assertTrue(bond.wait_until_broken(rospy.Duration(10.0)))


def main():
    rospy.init_node('exercise_bondpy', anonymous=True, disable_signals=True)
    rostest.run(PKG, 'exercise_bondpy', Exerciser, sys.argv)

if __name__ == '__main__': main()
