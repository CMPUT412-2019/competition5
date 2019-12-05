import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Empty
from smach import State
from typing import Callable, Optional

__all__ = ['InitAMCLState']


def pose_to_pose_with_covariance(pose):  # type: (PoseStamped) -> PoseWithCovarianceStamped
    pose_with_covariance = PoseWithCovarianceStamped()
    pose_with_covariance.header.frame_id = pose.header.frame_id
    pose_with_covariance.pose.pose = pose.pose
    return pose_with_covariance


class InitAMCLState(State):
    def __init__(self, initial_pose=None, clear_costmaps=True):  # type: (Optional[Callable[[], PoseStamped]], Optional[bool]) -> None
        super(InitAMCLState, self).__init__(outcomes=['ok'])
        self.initial_pose = initial_pose
        self.clear_costmaps = clear_costmaps

        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
        self.clear_costmaps_proxy = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

    def execute(self, ud):
        if self.initial_pose is not None:
            self.pose_pub.publish(pose_to_pose_with_covariance(self.initial_pose()))
        if self.clear_costmaps:
            self.clear_costmaps_proxy.call()

        return 'ok'
