#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('fibonacci_client_py')
    
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_action_server_to_start()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal()
    goal.order = 20

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_goal_to_finish()

    # Prints out the result of executing the action
    result = client.get_result()  # A FibonacciResult
    print "Result:", ', '.join([str(n) for n in result.sequence])
