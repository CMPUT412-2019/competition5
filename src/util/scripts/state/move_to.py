import numpy as np

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from ros_numpy import numpify
from smach import State
from tf import TransformListener, transformations

from src.util.scripts.util import SubscriberValue, notify_pushed, angle_diff


class MoveToState(State):
    def __init__(self, target, v):  # type: (List[ParkingSquare], ARCube, float) -> None
        super(MoveToState, self).__init__(outcomes=['ok'])
        self.v = v
        self.target = target
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/viz/moveto_target', PoseStamped, queue_size=1)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()

    def execute(self, ud):
        while not rospy.is_shutdown():
            target = self.target()
            self.target_pub.publish(target)

            try:
                this_pose = PoseStamped()
                this_pose.header.frame_id = 'odom'
                this_pose.pose = self.odometry.value.pose.pose
                this_pose = self.tf_listener.transformPose('map', this_pose)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
                continue

            this_position = numpify(this_pose.pose.position)[0:2]
            target_position = numpify(target.pose.position)[0:2]

            if np.linalg.norm(this_position - target_position) < 0.01:
                self.twist_pub.publish(Twist())
                break

            target_angle = np.arctan2(target.pose.position.y - this_pose.pose.position.y, target.pose.position.x - this_pose.pose.position.x)
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            t = Twist()
            t.linear.x = self.v
            t.angular.z = -4 * angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)

        while not rospy.is_shutdown():
            try:
                this_pose = PoseStamped()
                this_pose.header.frame_id = 'odom'
                this_pose.pose = self.odometry.value.pose.pose
                this_pose = self.tf_listener.transformPose('map', this_pose)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
                continue

            target_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(self.target().pose.orientation)))
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            if angle_diff(this_angle, target_angle) < 0.05:
                self.twist_pub.publish(Twist())
                return 'ok'

            t = Twist()
            t.angular.z = -2 * angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)