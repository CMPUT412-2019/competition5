import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from ros_numpy import numpify
from smach import State
from tf import TransformListener, transformations

from src.util.scripts.ar_tag import qv_mult
from src.util.scripts.util import SubscriberValue, angle_diff


class PushToGoalState(State):
    def __init__(self, target, v):  # type: (Callable[[], PoseStamped], float) -> None
        super(PushToGoalState, self).__init__(outcomes=['ok'], input_keys=['target_pose'])
        self.v = v
        self.target = target
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()

    def execute(self, ud):
        while True:
            target_pose = self.target()

            try:
                this_pose = PoseStamped()
                this_pose.header.frame_id = 'odom'
                this_pose.pose = self.odometry.value.pose.pose
                this_pose = self.tf_listener.transformPose('map', this_pose)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
                continue

            cube_offset = 0.17 + 0.32  # TODO: remove magic numbers
            this_position = numpify(this_pose.pose.position)[0:2]
            cube_position = this_position + qv_mult(numpify(this_pose.pose.orientation), [1, 0, 0])[0:2] * cube_offset
            target_position = numpify(target_pose.pose.position)[0:2]
            print(target_position - this_position, target_position - cube_position)
            if (np.dot(target_position - this_position, target_position - cube_position)) <= 0:
                return 'ok'

            target_angle = np.arctan2(target_pose.pose.position.y - this_pose.pose.position.y, target_pose.pose.position.x - this_pose.pose.position.x)
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            t = Twist()
            t.linear.x = self.v
            t.angular.z = - angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)
