import numpy as np
import rospy
import tf
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import Odometry
from ros_numpy import numpify, msgify
from smach import State, StateMachine, Sequence
from tf import TransformListener, transformations

from src.linefollow.scripts.state.rotate import RotateState
from src.util.scripts.ar_tag import qv_mult, ARCube
from src.util.scripts.parking_square import ParkingSquare
from src.util.scripts.state.function import ReturnFunctionState
from src.util.scripts.state.park_into_pose import park_into_pose
from src.util.scripts.util import SubscriberValue, angle_diff, notify_pushed, normal_to_quaternion


# class PushToMarkerSquareState(State):
#     def __init__(self, squares, cube, v):  # type: (List[ParkingSquare], ARCube, float) -> None
#         super(PushToMarkerSquareState, self).__init__(outcomes=['ok'])
#         self.v = v
#         self.cube = cube
#         self.squares = squares
#         self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
#         self.target_pub = rospy.Publisher('/viz/push_target', PoseStamped, queue_size=1)
#         self.cube_pub = rospy.Publisher('/viz/push_cube', PoseStamped, queue_size=1)
#         self.odometry = SubscriberValue('/odom', Odometry)
#         self.tf_listener = TransformListener()
#
#     def execute(self, ud):
#         while not rospy.is_shutdown():
#             target_pose = next(square.pose for square in self.squares if square.contains_marker())
#             self.target_pub.publish(target_pose)
#
#             try:
#                 this_pose = PoseStamped()
#                 this_pose.header.frame_id = 'odom'
#                 this_pose.pose = self.odometry.value.pose.pose
#                 this_pose = self.tf_listener.transformPose('map', this_pose)
#             except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
#                 continue
#
#             cube_offset = 0.2 + self.cube.cube_side_length/2  # TODO: remove magic numbers
#             this_position = numpify(this_pose.pose.position)[0:2]
#             cube_position = this_position + qv_mult(numpify(this_pose.pose.orientation), [1, 0, 0])[0:2] * cube_offset
#
#             cube_pose = PoseStamped()
#             cube_pose.header.frame_id = 'map'
#             cube_pose.pose.position = msgify(Point, np.append(cube_position, 0))
#             cube_pose.pose.orientation.w = 1
#             self.cube_pub.publish(cube_pose)
#
#             target_position = numpify(target_pose.pose.position)[0:2]
#             if (np.dot(target_position - this_position, target_position - cube_position)) <= 0:
#                 self.twist_pub.publish(Twist())
#                 notify_pushed()
#                 return 'ok'
#
#             target_angle = np.arctan2(target_pose.pose.position.y - this_pose.pose.position.y, target_pose.pose.position.x - this_pose.pose.position.x)
#             this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))
#
#             t = Twist()
#             t.linear.x = self.v
#             t.angular.z = -1.5 * angle_diff(this_angle, target_angle)
#             self.twist_pub.publish(t)
#
#
# def navigate_behind_cube(squares):  # type: (List[ParkingSquare]) -> StateMachine
#
#     def goal():
#         marker_square = next(square for square in squares if square.contains_marker())
#         cube_square = next(square for square in squares if square.contains_cube())
#
#         print('Square {} contains marker; square {} contains cube.'.format(marker_square.number, cube_square.number))
#         if cube_square.number > marker_square.number:
#             goal_number = cube_square.number + 1
#         else:
#             goal_number = cube_square.number - 1
#         print('Going to square {}'.format(goal_number))
#         goal_square = next(square for square in squares if square.number == goal_number)
#         return goal_square.pose
#
#     def direction():
#         marker_square = next(square for square in squares if square.contains_marker())
#         cube_square = next(square for square in squares if square.contains_cube())
#
#         print('Square {} contains marker; square {} contains cube.'.format(marker_square.number, cube_square.number))
#         if cube_square.number > marker_square.number:
#             direction = 'ccw'
#         else:
#             direction = 'cw'
#         print('Turning {}'.format(direction))
#         return direction
#
#     sm = StateMachine(outcomes=['ok', 'err'])
#     with sm:
#         StateMachine.add('PARK', park_into_pose(goal), transitions={'ok': 'CHOOSE_DIRECTION'})
#         StateMachine.add('CHOOSE_DIRECTION', ReturnFunctionState(direction, ['ccw', 'cw']), transitions={'ccw': 'TURN_CCW', 'cw': 'TURN_CW'})
#         StateMachine.add('TURN_CCW', RotateState(angle=np.pi/2))
#         StateMachine.add('TURN_CW', RotateState(angle=-np.pi/2))
#     return sm


class PushToGoalState(State):
    def __init__(self, cube, target, v):  # type: (ARCube, Callable[[], PoseStamped], float) -> None
        super(PushToGoalState, self).__init__(outcomes=['ok'], input_keys=['target_pose'])
        self.v = v
        self.cube = cube
        self.target = target
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/viz/push_target', PoseStamped, queue_size=1)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()

    def execute(self, ud):
        while True:
            target_pose = self.target()
            self.target_pub.publish(target_pose)

            try:
                this_pose = PoseStamped()
                this_pose.header.frame_id = 'odom'
                this_pose.pose = self.odometry.value.pose.pose
                this_pose = self.tf_listener.transformPose('map', this_pose)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
                continue

            cube_offset = 0.18 + self.cube.cube_side_length/2 + 0.3  # TODO: remove magic numbers
            this_position = numpify(this_pose.pose.position)[0:2]
            cube_position = this_position + qv_mult(numpify(this_pose.pose.orientation), [1, 0, 0])[0:2] * cube_offset
            target_position = numpify(target_pose.pose.position)[0:2]
            print(target_position - this_position, target_position - cube_position)
            if (np.dot(target_position - this_position, target_position - cube_position)) <= 0:
                self.twist_pub.publish(Twist())
                return 'ok'

            target_angle = np.arctan2(target_pose.pose.position.y - this_pose.pose.position.y, target_pose.pose.position.x - this_pose.pose.position.x)
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            t = Twist()
            t.linear.x = self.v
            t.angular.z = -2 * angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)


def parking_square_target(marker, offset):  # type: (ARTag, float) -> Callable[[], PoseStamped]
    def inner():
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position = marker.get_pose_with_offset([0., 0., offset]).pose.position
        goal.pose.orientation = msgify(Quaternion, normal_to_quaternion(-marker.surface_normal))
        return goal

    return inner



class NavigateToGoalState(State):
    def __init__(self):
        super(NavigateToGoalState, self).__init__(outcomes=['ok', 'err'], input_keys=['target_pose'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        pose = ud.target_pose  # type: PoseStamped
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.orientation
        self.client.send_goal(goal)
        if self.client.wait_for_result():
            return 'ok'
        else:
            return 'err'


class ChooseNewNavGoalState(State):
    def __init__(self, target, cube, back_distance):  # type: (Callable[[], PoseStamped], ARCube, float) -> None
        super(ChooseNewNavGoalState, self).__init__(outcomes=['ok'], input_keys=['target_pose'], output_keys=['target_pose'])
        self.target = target
        self.cube = cube
        self.back_distance = back_distance

    def execute(self, ud):
        cube_pose = self.cube.pose  # type: PoseStamped
        cube_position = np.array([cube_pose.pose.position.x, cube_pose.pose.position.y])

        tag_pose = self.target()
        tag_position = np.array([tag_pose.pose.position.x, tag_pose.pose.position.y])

        r_mo = transformations.unit_vector(cube_position - tag_position)
        goal_position = cube_position + self.back_distance * r_mo

        e_mo = transformations.unit_vector(np.append(r_mo, [0]))
        orientation_facing_marker = np.eye(4)
        orientation_facing_marker[0:3, 0:3] = np.column_stack((-e_mo, np.cross([0, 0, 1], -e_mo), [0, 0, 1]))
        orientation_facing_marker = transformations.quaternion_from_matrix(orientation_facing_marker)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = cube_pose.header.frame_id
        goal_pose.pose.position = Point(goal_position[0], goal_position[1], 0)
        goal_pose.pose.orientation = Quaternion(
            orientation_facing_marker[0],
            orientation_facing_marker[1],
            orientation_facing_marker[2],
            orientation_facing_marker[3],
        )
        ud.target_pose = goal_pose
        return 'ok'


def navigate_behind_cube(target, cube):
    sm = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sm:
        Sequence.add('choose', ChooseNewNavGoalState(target, cube, 0.7))
        Sequence.add('goto', NavigateToGoalState())
    return sm
