from geometry_msgs.msg import Quaternion
from ros_numpy import msgify
from smach import State

from src.navigation.scripts.navigate_to_moving_goal import NavigateToMovingGoalState
from src.util.scripts.ar_tag import ARTag
from src.util.scripts.util import normal_to_quaternion


class NavigateToMarkerState(State):
    """
    Moves the robot to 0.5 meters in front of the last known position of the AR tag of a given id. This position is
    updated by an ARTag.

    Required userdata
      - marker_id (int): The ID of the marker to look for.
    """
    def __init__(self, marker):  # type: (ARTag) -> None
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['marker_id'])
        self.marker = marker

    def execute(self, ud):

        def get_pose():
            goal_pose = self.marker.get_pose_with_offset((0, 0, 0.5))
            goal_pose.pose.orientation = msgify(Quaternion, normal_to_quaternion(-self.marker.surface_normal))
            return goal_pose

        return NavigateToMovingGoalState(get_pose).execute({})
