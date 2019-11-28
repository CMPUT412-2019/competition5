import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from ros_numpy import msgify
from smach import Sequence

from src.linefollow.scripts.state.forward import ForwardState
from src.linefollow.scripts.state.stop import StopState
from src.navigation.scripts.navigate_to_goal import NavigateToGoalState
from src.util.scripts.ar_tag import qv_mult


def park_into_pose(goal):  # type: (Callable[[], PoseStamped]) -> StateMachine
    offset = 0.5
    v = 0.2

    dt = offset / v

    def offset_goal():
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        goal_pose = goal()  # type: PoseStamped
        position = goal_pose.pose.position
        orientation = goal_pose.pose.orientation
        position += qv_mult(orientation, np.array([offset, 0.0, 0.0]))
        pose.pose.position = msgify(Point, position)
        pose.pose.orientation = msgify(Quaternion, orientation)
        return pose

    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sq:
        sq.add('MOVE_TO_OFFSET', NavigateToGoalState(offset_goal))
        sq.add('MOVE_FORWARD', ForwardState(v, dt))
        sq.add('STOP', StopState())
    return sq