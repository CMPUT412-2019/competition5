import rospy
from geometry_msgs.msg import Point, Quaternion, PoseStamped


def get_named_pose(name):  # type: (str) -> PoseStamped
    position = [float(x) for x in rospy.get_param('named_poses/{}/position'.format(name))]
    orientation = [float(x) for x in rospy.get_param('named_poses/{}/orientation'.format(name))]
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position = Point(*position)
    pose.pose.orientation = Quaternion(*orientation)
    return pose
