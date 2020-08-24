import rosgraph
import roslib.message
import rostopic
from .exceptions import ActionException
from .simple_action_client import SimpleActionClient
import socket


# Based on tool implementation at https://github.com/mcgill-robotics/rosaction
def _get_master():
    return rosgraph.Master('/rosaction')


def get_action_list():
    """Returns list of registered ROS actions.

    Returns:
        List of ROS action namespaces.
    """
    try:
        subscribed_topics = {x[0] for x in _get_master().getSystemState()[1]}
        return sorted(
            topic[:-5]
            for topic in subscribed_topics
            if topic.endswith('/goal') and
                (topic[:-5] + '/cancel') in subscribed_topics
        )
    except socket.error:
        raise ActionException('Can''t connect to ROS master')


def get_action_node(action):
    """Returns ROS node that handles given action.

    Args:
        action: ROS action name.

    Returns:
        Name of ROS node handling given action.
    """
    try:
        topic = next(iter(
            x
            for x in _get_master().getSystemState()[1]
            if x[0] == action + '/goal'
        ), None)

        if topic is None:
            return None

        return topic[1][0]
    except socket.error:
        raise ActionException('Can''t connect to ROS master')


def get_action_client(action):
    """Returns client for given ROS action.

    Args:
        action: ROS action name.

    Returns:
        Action client for given action name.
    """
    try:
        action_type = rostopic.get_topic_type(action + '/goal')[0]
    except rostopic.ROSTopicIOException as te:
        raise ActionException('Failed to get action goal type: %s' % te)

    if action_type is None:
        return None

    action_cls = roslib.message.get_message_class(action_type[:-4])

    return SimpleActionClient(action, action_cls)


_ACTION_GOAL_LEN = len('ActionGoal')
_ACTION_FEEDBACK_LEN = len('ActionFeedback')
_ACTION_RESULT_LEN = len('ActionResult')


def get_action_goal_type(action):
    """Returns type of action goal message.

    Args:
        action: ROS action name.

    Returns:
        Action goal message type or None if not found.
    """
    try:
        goal_type = rostopic.get_topic_type(action + '/goal')[0]
    except rostopic.ROSTopicIOException as te:
        raise ActionException('Failed to get action goal type: %s' % te)

    if goal_type is None:
        return None

    return goal_type[:-_ACTION_GOAL_LEN] + 'Goal'


def get_action_feedback_type(action):
    """Returns type of action feedback message.

    Args:
        action: ROS action name.

    Returns:
        Action feedback message type or None if not found.
    """
    try:
        feedback_type = rostopic.get_topic_type(action + '/feedback')[0]
    except rostopic.ROSTopicIOException as te:
        raise ActionException('Failed to get action feedback type: %s' % te)

    if feedback_type is None:
        return None

    return feedback_type[:-_ACTION_FEEDBACK_LEN] + 'Feedback'


def get_action_result_type(action):
    """Returns type of action result message.

    Args:
        action: ROS action name.

    Returns:
        Action result message type or None if not found.
    """
    try:
        result_type = rostopic.get_topic_type(action + '/result')[0]
    except rostopic.ROSTopicIOException as te:
        raise ActionException('Failed to get action result type: %s' % te)

    if result_type is None:
        return None

    return result_type[:-_ACTION_RESULT_LEN] + 'Result'
