import numpy as np
import rospy
import tf
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from ros_numpy import numpify, msgify
from smach import State
from tf import TransformListener, transformations

from src.navigation.scripts.move_to import move_to
from src.util.scripts.ar_tag import qv_mult, ARCube, ARTag
from src.util.scripts.util import SubscriberValue, angle_diff, normal_to_quaternion


class PushToGoalState(State):
    def __init__(self, cube, target, v):  # type: (ARCube, Callable[[], PoseStamped], float) -> None
        super(PushToGoalState, self).__init__(outcomes=['ok'], input_keys=['target_pose'])
        self.v = v
        self.cube = cube
        self.target = target
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/viz/push_target', PoseStamped, queue_size=1)
        self.cube_pub = rospy.Publisher('/viz/push_cube', PoseStamped, queue_size=1)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()

    def execute(self, ud):
        while not rospy.is_shutdown():
            target_pose = self.target()
            self.target_pub.publish(target_pose)

            try:
                this_pose = PoseStamped()
                this_pose.header.frame_id = 'odom'
                this_pose.pose = self.odometry.value.pose.pose
                this_pose = self.tf_listener.transformPose('map', this_pose)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
                continue

            cube_offset = 0.2 + self.cube.cube_side_length/2  # TODO: remove magic numbers
            this_position = numpify(this_pose.pose.position)[0:2]
            cube_position = this_position + qv_mult(numpify(this_pose.pose.orientation), [1, 0, 0])[0:2] * cube_offset

            cube_pose = PoseStamped()
            cube_pose.header.frame_id = 'map'
            cube_pose.pose.position = msgify(Point, np.append(cube_position, 0))
            cube_pose.pose.orientation.w = 1
            self.cube_pub.publish(cube_pose)

            target_position = numpify(target_pose.pose.position)[0:2]
            if (np.dot(target_position - this_position, target_position - cube_position)) <= 0:
                self.twist_pub.publish(Twist())
                return 'ok'

            target_angle = np.arctan2(target_pose.pose.position.y - this_pose.pose.position.y, target_pose.pose.position.x - this_pose.pose.position.x)
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            t = Twist()
            t.linear.x = self.v
            t.angular.z = -1.5 * angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)


class NavigateBehindCubeState(State):
    def __init__(self, target, cube, back_distance):  # type: (Callable[[], PoseStamped], ARCube, float) -> None
        super(NavigateBehindCubeState, self).__init__(outcomes=['ok', 'err'])
        self.target = target
        self.cube = cube
        self.back_distance = back_distance
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        cube_pose = self.cube.pose  # type: PoseStamped
        cube_position = np.array([cube_pose.pose.position.x, cube_pose.pose.position.y])

        tag_pose = self.target()
        tag_position = np.array([tag_pose.pose.position.x, tag_pose.pose.position.y])

        r_mo = transformations.unit_vector(cube_position - tag_position)
        goal_position = cube_position + self.back_distance * r_mo
        orientation_facing_marker = normal_to_quaternion(-r_mo)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = cube_pose.header.frame_id
        goal_pose.pose.position = Point(goal_position[0], goal_position[1], 0)
        goal_pose.pose.orientation = msgify(Quaternion, orientation_facing_marker)
        if self._move_to(goal_pose):
            return 'ok'
        else:
            return 'err'

    def _move_to(self, pose):  # type: (PoseStamped) -> bool
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.orientation
        self.client.send_goal(goal)
        return self.client.wait_for_result()


class LineUpBehindCubeState(State):
    def __init__(self, cube, v):  # type: (ARCube, float) -> None
        super(LineUpBehindCubeState, self).__init__(outcomes=['ok', 'err'])
        self.cube = cube
        self.v = v
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.pose2d = SubscriberValue('/pose2d', Pose2D)
        self.tf_listener = TransformListener()
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        rate = rospy.Rate(10)

        print('LineUp: moving to start')

        pose2d = self.pose2d.value  # type: Pose2D
        robot_angle = pose2d.theta
        robot_normal = np.array([np.cos(robot_angle), np.sin(robot_angle)])
        cube_normals = [
            np.array([self.cube.surface_normal[0], self.cube.surface_normal[1]]),
            np.array([-self.cube.surface_normal[0], self.cube.surface_normal[1]]),
            np.array([-self.cube.surface_normal[0], -self.cube.surface_normal[1]]),
            np.array([self.cube.surface_normal[0], -self.cube.surface_normal[1]]),
        ]
        best_fit = np.argmax([np.dot(robot_normal, cube_normal) for cube_normal in cube_normals])
        target_normal = cube_normals[best_fit]
        target_normal = np.array([target_normal[0], target_normal[1], 0.0])
        target_pose = self.cube.pose
        target_pose.pose.position = msgify(Point, numpify(target_pose.pose.position) - 0.7*target_normal)
        target_pose.pose.position.z = 0.0
        target_orientation = normal_to_quaternion(target_normal)
        target_pose.pose.orientation = msgify(Quaternion, target_orientation)

        if not move_to(self.client, target_pose):
            return 'err'

        print('LineUp: moving forward')
        start_time = rospy.get_time()
        duration = (0.7 - self.cube.cube_side_length / 2) / self.v
        while not rospy.is_shutdown() and rospy.get_time() - start_time < duration:
            t = Twist()
            t.linear.x = self.v
            self.twist_pub.publish(t)
            rate.sleep()
        return 'ok'


def parking_square_target(marker, offset):  # type: (ARTag, float) -> Callable[[], PoseStamped]
    def inner():
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position = marker.get_pose_with_offset([0., 0., offset]).pose.position
        goal.pose.orientation = msgify(Quaternion, normal_to_quaternion(-marker.surface_normal))
        return goal

    return inner
