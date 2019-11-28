import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction
from smach import State

from src.navigation.scripts.move_to import move_to


class NavigateToNamedPoseState(State):
    """
    Moves the robot to a predefined pose. The poses are read from `named_poses/{pose_name}` on the parameter server,
    and are defined in the map frame.
    """
    def __init__(self, pose_name):  # type: (str) -> None
        State.__init__(self, outcomes=['ok', 'err'])
        position = [float(x) for x in rospy.get_param('named_poses/{}/position'.format(pose_name))]
        orientation = [float(x) for x in rospy.get_param('named_poses/{}/orientation'.format(pose_name))]
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.pose.position = Point(*position)
        self.pose.pose.orientation = Quaternion(*orientation)
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        return 'ok' if move_to(self.client, self.pose) else 'err'
