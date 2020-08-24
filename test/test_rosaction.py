#!/usr/bin/env python

from contextlib import contextmanager
from actionlib import rosaction
from actionlib.exceptions import ActionException
import os
import sys
import unittest
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import rospy
import rostest


@contextmanager
def capture_stdout():
    real_stdout = sys.stdout
    fake_stdout = StringIO()
    sys.stdout = fake_stdout
    try:
        yield fake_stdout
    finally:
        sys.stdout = real_stdout


@contextmanager
def offline():
    uri = os.environ['ROS_MASTER_URI']
    os.environ['ROS_MASTER_URI'] = 'http://fake_host:12345'
    try:
        yield
    finally:
        os.environ['ROS_MASTER_URI'] = uri


def striplines(s):
    return '\n'.join(l.strip() for l in s.split('\n'))


class TestRosaction(unittest.TestCase):
    ACTIONS = [
        '/reference_action',
        '/foo/reference_action',
        '/bar/reference_action',
    ]

    def test_get_action_list_returns_list_of_registered_actions(self):
        with capture_stdout() as f:
            rosaction.main(['list'])

            actions = f.getvalue().strip().split()
            self.assertItemsEqual(self.ACTIONS, actions)

    def test_get_action_list_offline_raises_exception(self):
        with offline():
            with self.assertRaises(ActionException):
                rosaction.main(['list'])

    def test_get_action_goal_type(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['type', action, 'goal'])

                goal_type = f.getvalue().strip()
                self.assertEqual('actionlib/TestGoal', goal_type)

    def test_get_action_goal_type_offline_raises_exception(self):
        with offline():
            with self.assertRaises(ActionException):
                rosaction.main(['type', '/reference_action', 'goal'])

    def test_get_action_goal_type_with_unknown_action_prints_error(self):
        with capture_stdout() as f:
            rosaction.main(['type', '/unknown_action', 'goal'])

            self.assertEqual('Can''t find action "/unknown_action"',
                             f.getvalue().strip())

    def test_get_action_type_defaults_to_goal(self):
        with capture_stdout() as f:
            rosaction.main(['type', '/unknown_action', 'goal'])
            get_type_goal_result = f.getvalue().strip()

        with capture_stdout() as f:
            rosaction.main(['type', '/unknown_action'])
            get_type_result = f.getvalue().strip()

        self.assertEqual(get_type_goal_result, get_type_result)

    def test_get_action_feedback_type(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['type', action, 'feedback'])

                goal_type = f.getvalue().strip()
                self.assertEqual('actionlib/TestFeedback', goal_type)

    def test_get_action_feedback_type_offline_raises_exception(self):
        with offline():
            with self.assertRaises(ActionException):
                rosaction.main(['type', '/reference_action', 'feedback'])

    def test_get_action_feedback_type_with_unknown_action_prints_error(self):
        with capture_stdout() as f:
            rosaction.main(['type', '/unknown_action', 'feedback'])

            self.assertEqual('Can''t find action "/unknown_action"',
                             f.getvalue().strip())

    def test_get_action_result_type(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['type', action, 'result'])

                goal_type = f.getvalue().strip()
                self.assertEqual('actionlib/TestResult', goal_type)

    def test_get_action_result_type_offline_raises_exception(self):
        with offline():
            with self.assertRaises(ActionException):
                rosaction.main(['type', '/reference_action', 'result'])

    def test_get_action_result_type_with_unknown_action_prints_error(self):
        with capture_stdout() as f:
            rosaction.main(['type', '/unknown_action', 'result'])

            self.assertEqual('Can''t find action "/unknown_action"',
                             f.getvalue().strip())

    def test_get_action_info_returns_action_node(self):
        with capture_stdout() as f:
            rosaction.main(['info', '/reference_action'])

            self.assertIn('Node: /ref_server', f.getvalue().split('\n'))

        with capture_stdout() as f:
            rosaction.main(['info', '/foo/reference_action'])

            self.assertIn('Node: /foo/ref_server', f.getvalue().split('\n'))

        with capture_stdout() as f:
            rosaction.main(['info', '/bar/reference_action'])

            self.assertIn('Node: /bar/ref_server', f.getvalue().split('\n'))

    def test_get_action_info_returns_action_goal_type(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['info', action])

                self.assertIn('Type: actionlib/TestGoal', f.getvalue().split('\n'))

    def test_get_action_info_offline_raises_exception(self):
        with offline():
            with self.assertRaises(ActionException):
                rosaction.main(['info', '/reference_action'])

    def test_get_action_info_with_unknown_action_prints_error(self):
        with capture_stdout() as f:
            rosaction.main(['info', '/unknown_action'])

            self.assertEqual('Can''t find action "/unknown_action"',
                             f.getvalue().strip())

    def test_send_action_goal(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['send', action, 'goal: 1'])

                self.assertEqual('SUCCEEDED\n---\nresult: 0', f.getvalue().strip())

    def test_send_action_goal_feedback(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['send', action, 'goal: 9'])

                self.assertEqual(striplines('''
                    feedback: 0
                    ---
                    feedback: 1
                    ---
                    feedback: 2
                    ---
                    SUCCEEDED
                    ---
                    result: 3
                '''.strip()), f.getvalue().strip())

    def test_send_action_goal_no_wait(self):
        for action in self.ACTIONS:
            with capture_stdout() as f:
                rosaction.main(['send', action, 'goal: 9', '--no-wait'])

                self.assertNotIn('result: 3', f.getvalue().strip())

    def test_send_action_goal_offline_raises_exception(self):
        with offline():
            with self.assertRaises(ActionException):
                rosaction.main(['send', '/reference_action', 'goal: 1'])

    def test_send_action_goal_with_unknown_action_prints_error(self):
        with capture_stdout() as f:
            rosaction.main(['send', '/unknown_action', 'goal: 1'])

            self.assertEqual('Can''t find action "/unknown_action"',
                             f.getvalue().strip())


if __name__ == '__main__':
    rospy.init_node('rosaction_test')
    rostest.run('actionlib', 'test_rosaction', TestRosaction, sys.argv)
