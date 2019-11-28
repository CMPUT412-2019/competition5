import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseGoal


def move_to(client, pose):  # type: (SimpleActionClient, PoseStamped) -> bool
    client.wait_for_server()
    twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
    tries = 1
    while not rospy.is_shutdown() and tries <= 3:
        tries += 1
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.orientation
        client.send_goal(goal)
        if client.wait_for_result():
            twist_pub.unregister()
            return True
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start_time < 0.5:
            t = Twist()
            t.linear.x = -0.5
            twist_pub.publish(t)
    twist_pub.unregister()
    return False
