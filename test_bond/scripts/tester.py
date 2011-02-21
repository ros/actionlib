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

# This is largely copied from bondpy, with hooks that "break" it for testing purposes.

import threading
import time
import uuid

import roslib; roslib.load_manifest('test_bond')
import rospy
from bond.msg import *
from test_bond.srv import *

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

import BondSM_sm

class Timeout:
    def __init__(self, duration, on_timeout=None):
        self.duration = duration
        self.timer = threading.Timer(0, self._on_timer)
        self.deadline = rospy.Time.now()
        self.on_timeout = on_timeout

    def reset(self):
        self.timer.cancel()
        self.timer = threading.Timer(self.duration.to_sec(), self._on_timer)
        self.timer.start()
        self.deadline = rospy.Time.now() + self.duration
        return self

    def cancel(self):
        self.timer.cancel()

    def left(self):
        return max(rospy.Duration(0), self.deadline - rospy.Time.now())

    def _on_timer(self):
        if self.on_timeout:
            self.on_timeout()


class BondTester:
    def __init__(self, req):
        self.req = req
        self.topic = req.topic
        self.id = req.id
        self.instance_id = str(uuid.uuid4())
        self.on_death = None
        self.on_life = None
        self.is_shutdown = False
        self.sister_died_first = False
        self.death_started = None

        self.sm = BondSM_sm.BondSM_sm(self)
        #self.sm.setDebugFlag(True)
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)

        self.connect_timeout = Timeout(rospy.Duration(Constants.DEFAULT_CONNECT_TIMEOUT), self._on_connect_timeout)
        self.heartbeat_timeout = Timeout(rospy.Duration(Constants.DEFAULT_HEARTBEAT_TIMEOUT), self._on_heartbeat_timeout)
        self.disconnect_timeout = Timeout(rospy.Duration(Constants.DEFAULT_DISCONNECT_TIMEOUT), self._on_disconnect_timeout)

        self.connect_timeout.reset()

        self.sub = rospy.Subscriber(self.topic, Status, self._on_bond_status)
        self.pub = rospy.Publisher(self.topic, Status)

        self.thread = threading.Thread(target=self._publishing_thread)
        self.thread.daemon = True
        self.thread.start()

        if req.delay_death >= rospy.Duration(0.0):
            self.death_timeout = Timeout(req.delay_death, self.die).reset()

    def _on_connect_timeout(self):
        with self.lock:
            self.sm.ConnectTimeout()
    def _on_heartbeat_timeout(self):
        with self.lock:
            self.sm.HeartbeatTimeout()
    def _on_disconnect_timeout(self):
        with self.lock:
            self.sm.DisconnectTimeout()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        if not self.is_shutdown:
            with self.lock:
                self.is_shutdown = True
                if self.sm.getState().getName() != 'SM.Dead':
                    print "I'm not dead yet:", self.id, " in ", self.sm.getState().getName()
                    self.die()
                self.sub.unregister()
                self.pub.unregister()
                self.condition.notify_all()
                print "Unregistered"

    def _on_bond_status(self, msg):
        # Filters out messages from other bonds and messages from ourselves
        if msg.id == self.id and msg.instance_id != self.instance_id:
            with self.lock:
                if msg.active or self.req.inhibit_death:
                    self.sm.SisterAlive()
                else:
                    self.sm.SisterDead()

                    # Immediate ack for sister's death notification
                    if self.sister_died_first:
                        self._publish(False)


    def _publish(self, active):
        msg = Status()
        msg.header.stamp = rospy.Time.now()
        msg.id = self.id
        msg.instance_id = self.instance_id
        msg.active = active

        if not msg.active and self.req.inhibit_death_message:
            pass
        else:
            self.pub.publish(msg)

    def _publishing_thread(self):

        time.sleep(self.req.delay_connect.to_sec())

        with self.lock:
            # Publishing ALIVE
            while not self.is_shutdown and self.sm.getState().getName() in ['SM.WaitingForSister', 'SM.Alive']:
                self._publish(True)
                self.condition.wait(Constants.DEFAULT_HEARTBEAT_PERIOD)

            # Publishing DEAD
            while not self.is_shutdown and self.sm.getState().getName() == 'SM.AwaitSisterDeath':
                self._publish(False)
                self.condition.wait(Constants.DEAD_PUBLISH_PERIOD)


    def Connected(self):
        self.connect_timeout.cancel()
        self.condition.notify_all()
        if self.on_life:
            self.on_life()

    def Heartbeat(self):
        self.heartbeat_timeout.reset()

    def SisterDied(self):
        self.sister_died_first = True

    def Death(self):
        self.condition.notify_all()
        self.heartbeat_timeout.cancel()
        self.disconnect_timeout.cancel()
        if not self.death_started:
            self.death_started = rospy.Time.now()

    def StartDying(self):
        self.heartbeat_timeout.cancel()
        self.disconnect_timeout.reset()
        if not self.death_started:
            self.death_started = rospy.Time.now()

    def wait_for_life(self, timeout = None):
        deadline = timeout and Timeout(timeout).reset()
        with self.lock:
            while self.sm.getState().getName() == 'SM.WaitingForSister':
                if deadline and deadline.left() == rospy.Duration(0):
                    break
                self.condition.wait(deadline and deadline.left().to_sec())
            return self.sm.getState().getName() != 'SM.WaitingForSister'

    def wait_for_death(self, timeout = None):
        deadline = timeout and Timeout(timeout).reset()
        with self.lock:
            while self.sm.getState().getName() != 'SM.Dead':
                if deadline and deadline.left() == rospy.Duration(0):
                    break
                self.condition.wait(deadline and deadline.left().to_sec())
            return self.sm.getState().getName() == 'SM.Dead'

    def is_dead(self):
        with self.lock:
            return self.sm.getState().getName() == 'SM.Dead'

    def die(self):
        with self.lock:
            self.sm.Die()
            self._publish(False)

    def __repr__(self):
        return "[Bond %s, Instance %s (%s)]" % (self.id, self.instance_id, self.sm.getState().getName())


class Tester:
    def __init__(self):
        self.bond_tester = None
        self.service = rospy.Service('test_bond', TestBond, self._test_bond)

    def _test_bond(self, req):
        print "TEST"
        if self.bond_tester:
            self.bond_tester.shutdown()
        self.bond_tester = BondTester(req)
        print "Test bond instance id: %s" % self.bond_tester.instance_id
        return TestBondResponse()

def main():
    rospy.init_node('bond_tester', anonymous=True, disable_signals=True)
    tester = Tester()
    rospy.spin()

if __name__ == '__main__': main()
