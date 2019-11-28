import numpy as np
import rospy
import tf
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from ros_numpy import numpify, msgify
from smach import State, StateMachine, Sequence
from tf import TransformListener, transformations
from typing import List

from src.linefollow.scripts.state.rotate import RotateState
from src.navigation.scripts.move_to import move_to
from src.util.scripts.ar_tag import qv_mult, ARCube
from src.util.scripts.parking_square import ParkingSquare, closest_square
from src.util.scripts.state.function import ReturnFunctionState
from src.util.scripts.state.park_into_pose import park_into_pose
from src.util.scripts.util import SubscriberValue, angle_diff, normal_to_quaternion


class PushToMarkerSquareState(State):
    def __init__(self, squares, v):  # type: (List[ParkingSquare], float) -> None
        super(PushToMarkerSquareState, self).__init__(outcomes=['ok'])
        self.v = v
        self.squares = squares
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()

    def execute(self, ud):
        while not rospy.is_shutdown():
            target_pose = next(square.pose for square in self.squares if square.contains_marker())

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
            if (np.dot(target_position - this_position, target_position - cube_position)) <= 0:
                return 'ok'

            target_angle = np.arctan2(target_pose.pose.position.y - this_pose.pose.position.y, target_pose.pose.position.x - this_pose.pose.position.x)
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            t = Twist()
            t.linear.x = self.v
            t.angular.z = - angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)


def navigate_behind_cube(squares):  # type: (List[ParkingSquare]) -> StateMachine

    def goal():
        marker_square = next(square for square in squares if square.contains_marker())
        cube_square = next(square for square in squares if square.contains_cube())

        print('Square {} contains marker; square {} contains cube.'.format(marker_square.number, cube_square.number))
        if cube_square.number > marker_square.number:
            goal_number = cube_square.number + 1
        else:
            goal_number = cube_square.number - 1
        print('Going to square {}'.format(goal_number))
        goal_square = next(square for square in squares if square.number == goal_number)
        return goal_square.pose

    def direction():
        marker_square = next(square for square in squares if square.contains_marker())
        cube_square = next(square for square in squares if square.contains_cube())

        print('Square {} contains marker; square {} contains cube.'.format(marker_square.number, cube_square.number))
        if cube_square.number > marker_square.number:
            direction = 'ccw'
        else:
            direction = 'cw'

    sm = StateMachine(outcomes=['ok', 'err'])
    with sm:
        StateMachine.add('PARK', park_into_pose(goal), transitions={'ok': 'CHOOSE_DIRECTION'})
        StateMachine.add('CHOOSE_DIRECTION', ReturnFunctionState(direction, ['ccw', 'cc']), transitions={'ccw': 'TURN_CCW', 'cw': 'TURN_CW'})
        StateMachine.add('TURN_CCW', RotateState(angle=np.pi/2))
        StateMachine.add('TURN_CW', RotateState(angle=-np.pi/2))
    return sm

