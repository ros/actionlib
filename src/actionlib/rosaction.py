from __future__ import print_function
import argparse
import rospy
import rosmsg
import actionlib
import actionlib.utils
from actionlib_msgs.msg import GoalStatus
import roslib.message
import yaml


GOAL_STATUSES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST"
}

def print_action_list(args):
    for action in actionlib.utils.get_action_list():
        print(action)


def print_action_info(args):
    goal_type = actionlib.utils.get_action_goal_type(args.action)
    if goal_type is None:
        print('Can''t find action "%s"' % args.action)
        return 1

    print('Node: {}'.format(actionlib.utils.get_action_node(args.action)))
    print('Type: {}'.format(goal_type))


def print_action_type(args):
    if args.subtype == 'goal':
        the_type = actionlib.utils.get_action_goal_type(args.action)
    elif args.subtype == 'feedback':
        the_type = actionlib.utils.get_action_feedback_type(args.action)
    elif args.subtype == 'result':
        the_type = actionlib.utils.get_action_result_type(args.action)

    if the_type is None:
        print('Can''t find action "%s"' % args.action)
        return 1

    print(the_type)


def send_action_goal(args):
    client = actionlib.utils.get_action_client(args.action)
    if client is None:
        print('Can''t find action "%s"' % args.action)
        return 1

    client.wait_for_server(rospy.Duration(args.timeout))

    goal_type = actionlib.utils.get_action_goal_type(args.action)
    goal_cls = roslib.message.get_message_class(goal_type)
    goal = goal_cls()
    roslib.message.fill_message_args(goal, [yaml.load(args.goal)])

    def print_feedback(feedback):
        print(feedback)
        print('---')

    def print_result(status, result):
        print(GOAL_STATUSES.get(status, status))
        print('---')
        print(result)

    client.send_goal(goal, feedback_cb=print_feedback, done_cb=print_result)

    if not args.no_wait:
        def cancel_action():
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(args.timeout))

        rospy.on_shutdown(cancel_action)
        client.wait_for_result(rospy.Duration(args.timeout))


def main(args):
    parser = argparse.ArgumentParser()

    commands = parser.add_subparsers()
    list_command = commands.add_parser('list')
    list_command.set_defaults(func=print_action_list)

    info_command = commands.add_parser('info')
    info_command.add_argument('action', help='Action namespace')
    info_command.set_defaults(func=print_action_info)

    type_command = commands.add_parser('type')
    type_command.add_argument('action', help='Action namespace')
    type_command.add_argument('subtype', choices=['goal', 'feedback', 'result'],
                              default='goal', nargs='?')
    type_command.set_defaults(func=print_action_type)

    send_command = commands.add_parser('send')
    send_command.add_argument('--timeout', type=int, default=30,
                              help='Timeout in seconds to wait for result')
    send_command.add_argument('--no-wait', action='store_true', default=False,
                              help='Do not wait for result')
    send_command.add_argument('action', help='Action namespace')
    send_command.add_argument('goal', help='Action goal in YAML format')
    send_command.set_defaults(func=send_action_goal)

    parsed_args = parser.parse_args(args)
    return parsed_args.func(parsed_args)
