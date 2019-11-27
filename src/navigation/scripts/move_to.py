from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal


def move_to(client, pose):  # type: (SimpleActionClient, PoseStamped) -> bool
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = pose.header.frame_id
    goal.target_pose.pose.position = pose.pose.position
    goal.target_pose.pose.orientation = pose.pose.orientation
    client.send_goal(goal)
    return client.wait_for_result()
