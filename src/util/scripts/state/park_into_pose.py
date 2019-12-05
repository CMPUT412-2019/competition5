import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from ros_numpy import msgify, numpify
from smach import Sequence

from src.linefollow.scripts.state.forward import ForwardState
from src.linefollow.scripts.state.stop import StopState
from src.navigation.scripts.navigate_to_goal import NavigateToGoalState
from src.util.scripts.ar_tag import qv_mult
from src.util.scripts.state.move_to import MoveToState

from typing import Callable


def offset_goal(goal, _offset):  # type: (Callable[[], PoseStamped], float) -> Callable[[], PoseStamped]
    def inner():
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        goal_pose = goal()  # type: PoseStamped
        position = numpify(goal_pose.pose.position)
        orientation = numpify(goal_pose.pose.orientation)
        position += qv_mult(orientation, np.array([-_offset, 0.0, 0.0]))
        pose.pose.position = msgify(Point, position)
        pose.pose.orientation = msgify(Quaternion, orientation)
        return pose

    return inner


def park_into_pose(goal, offset=0.5):  # type: (Callable[[], PoseStamped]) -> StateMachine
    v = 0.2
    end_offset = 0.09

    dt = offset / v

    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sq:
        sq.add('MOVE_TO_OFFSET', NavigateToGoalState(offset_goal(goal, offset)))
        # sq.add('MOVE_FORWARD', ForwardState(v, dt))
        sq.add('MOVE_FORWARD', MoveToState(offset_goal(goal, -end_offset), v))
        sq.add('STOP', StopState())
    return sq