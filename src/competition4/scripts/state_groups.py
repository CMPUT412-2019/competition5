import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from ros_numpy import msgify, numpify
from smach import Sequence, StateMachine
import rospy

from src.boxpush.scripts.state.boxpush import navigate_behind_cube, PushToGoalState, \
    parking_square_target
from src.util.scripts.parking_square import ParkingSquare
from src.competition4.scripts.state.shape_search import SearchForShapeInSquareState, ShortCircuitParkingSquareState
from src.util.scripts.state.park_into_pose import park_into_pose
from src.linefollow.scripts.filter import RedDetector, RedLowerDetector, white_filter, red_filter
from src.linefollow.scripts.pid_control import PIDController
from src.linefollow.scripts.state.find_marker import FindMarkerState
from src.linefollow.scripts.state.forward import ForwardState
from src.linefollow.scripts.state.linefollow import LineFollowState, TransitionAfter, TransitionAt
from src.linefollow.scripts.state.location1 import Location1State
from src.linefollow.scripts.state.location2 import Location2State
from src.linefollow.scripts.state.location3 import Location3State
from src.linefollow.scripts.state.rotate import RotateState
from src.linefollow.scripts.state.stop import StopState
from src.navigation.scripts.navigate_to_marker import NavigateToMarkerState
from src.navigation.scripts.navigate_to_named_pose import NavigateToNamedPoseState
from src.util.scripts.ar_tag import ARTag, ARCube
from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.util.scripts.state.absorb_result import AbsorbResultState
from src.util.scripts.state.function import FunctionState, ReturnFunctionState
from src.util.scripts.util import ProximityDetector, notify_artag, notify_unmarked, notify_pushed
from src.util.scripts.select_number import SelectNumberState

forward_speed = 0.8
# forward_speed = 0.8
kp = 9.
kd = -0.01 # 0.02
ki = 1.

proximity_detector = ProximityDetector(.7)


def location1(cam_pixel_to_point):  # type: (CamPixelToPointServer) -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        Sequence.add('TURN_L', RotateState(np.pi/2))
        Sequence.add('LOCATION1', Location1State(cam_pixel_to_point), transitions={'err': 'TURN_R'})
        Sequence.add('TURN_R', RotateState(-np.pi/2))
    return sq


def move_to_stop_line(lower=False):  # type: (bool) -> StateMachine
    detector = RedDetector() if not lower else RedLowerDetector()
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        # Forward until stop line
        Sequence.add('FOLLOW', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                               TransitionAfter(detector)))
        Sequence.add('STOP', StopState())
    return sq


def enter_split():  # type: () -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        Sequence.add('FOLLOW_W', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                                 TransitionAt(RedDetector())))
        Sequence.add('FOLLOW_R', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter,
                                                 TransitionAfter(RedDetector())))
        Sequence.add('STOP', StopState())
        Sequence.add('TURN', RotateState(np.pi / 2 * 2 / 3))
    return sq


def move_to_obstacle():  # type: () -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        Sequence.add('FOLLOW', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                               TransitionAt(proximity_detector)))
        Sequence.add('STOP', StopState())
    return sq


def location2():  # type: () -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok', output_keys=['green_shape'])
    with sq:
        Sequence.add('LOCATION2', Location2State(), transitions={'err': 'TURN'})
        Sequence.add('TURN', RotateState(np.pi))
    return sq


def exit_split():  # type: () -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        Sequence.add('FOLLOW_W', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                                 TransitionAt(RedDetector())))
        Sequence.add('FOLLOW_R', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter,
                                                 TransitionAfter(RedDetector())))
        Sequence.add('TURN', RotateState(np.pi / 2 - 0.2))
    return sq


def off_ramp():  # type: () -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        Sequence.add('GOAL_OFFRAMP_START', NavigateToNamedPoseState('off_ramp_start'), transitions={'err': 'GOAL_OFFRAMP_END'})
        Sequence.add('GOAL_OFFRAMP_END', NavigateToNamedPoseState('off_ramp_end'), transitions={'err': 'ABSORB'})
        Sequence.add('ABSORB', AbsorbResultState())
    return sq


def on_ramp():  # type: () -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        Sequence.add('GOAL_ONRAMP', NavigateToNamedPoseState('on_ramp'), transitions={'err': 'ABSORB'})
        Sequence.add('ABSORB', AbsorbResultState())
    return sq


def ar_tag(marker):  # type: (ARTag) -> StateMachine
        sq = Sequence(outcomes=['ok'], connector_outcome='ok')
        with sq:
            # Sequence.add('AR_START', NavigateToNamedPoseState('ar_start'), transitions={'err': 'ABSORB'})
            # Sequence.add('AR_FIND', FindMarkerState(marker, 'cmd_vel_mux/input/teleop'))
            Sequence.add('AR_GOTO', NavigateToMarkerState(marker), transitions={'err': 'ABSORB'})
            Sequence.add('NOTIFY', FunctionState(notify_artag))
            Sequence.add('ABSORB', AbsorbResultState())
        return sq

#
# def joystick_location():  # type: () -> StateMachine
#         sq = Sequence(outcomes=['ok'], connector_outcome='ok')
#         with sq:
#             Sequence.add('SELECT_NUMBER', SelectNumberState(min=1, max=8))
#             Sequence.add('GOAL', NavigateToNumberState(), transitions={'err': 'ABSORB'})
#             Sequence.add('NOTIFY', FunctionState(notify_unmarked))
#             Sequence.add('ABSORB', AbsorbResultState())
#         return sq


def location3(cam_pixel_to_point):  # type: (CamPixelToPointServer) -> StateMachine
    sm = StateMachine(outcomes=['ok'], input_keys=['green_shape'])
    with sm:
        StateMachine.add('MOVETO1', move_to_stop_line(lower=True), transitions={'ok': 'TURN1_1'})
        StateMachine.add('TURN1_1', RotateState(np.pi / 2), transitions={'ok': 'LOCATION3_1'})
        StateMachine.add('LOCATION3_1', Location3State(cam_pixel_to_point), transitions={'ok': 'TURN1_2', 'err': 'TURN1_2', 'match': 'EXIT_TURN1'})
        StateMachine.add('TURN1_2', RotateState(-np.pi / 2), transitions={'ok': 'MOVETO2'})

        StateMachine.add('MOVETO2', move_to_stop_line(lower=True), transitions={'ok': 'TURN2_1'})
        StateMachine.add('TURN2_1', RotateState(np.pi / 2), transitions={'ok': 'LOCATION3_2'})
        StateMachine.add('LOCATION3_2', Location3State(cam_pixel_to_point), transitions={'ok': 'TURN2_2', 'err': 'TURN2_2', 'match': 'EXIT_TURN2'})
        StateMachine.add('TURN2_2', RotateState(-np.pi / 2), transitions={'ok': 'MOVETO3'})

        StateMachine.add('MOVETO3', move_to_stop_line(lower=True), transitions={'ok': 'TURN3_1'})
        StateMachine.add('TURN3_1', RotateState(np.pi / 2), transitions={'ok': 'LOCATION3_3'})
        StateMachine.add('LOCATION3_3', Location3State(cam_pixel_to_point), transitions={'ok': 'TURN3_2', 'err': 'TURN3_2', 'match': 'EXIT_TURN3'})
        StateMachine.add('TURN3_2', RotateState(-np.pi / 2))

        StateMachine.add('EXIT_TURN1', RotateState(-np.pi / 2), transitions={'ok': 'EXIT_MOVETO2'})
        StateMachine.add('EXIT_TURN2', RotateState(-np.pi / 2), transitions={'ok': 'EXIT_MOVETO3'})
        StateMachine.add('EXIT_TURN3', RotateState(-np.pi / 2))
        StateMachine.add('EXIT_MOVETO2', move_to_stop_line(lower=True), transitions={'ok': 'EXIT_MOVETO3'})
        StateMachine.add('EXIT_MOVETO3', move_to_stop_line(lower=True))
    return sm


def find_shape(squares, cam_pixel_to_point):  # type: (List[ParkingSquare], CamPixelToPointServer) -> StateMachine
    def found_shape():
        return 'found' if any(s.contains_shape() for s in squares) else 'ok'

    sq = Sequence(outcomes=['ok'], connector_outcome='ok', input_keys=['green_shape'])
    with sq:
        Sequence.add('SQUARE1', look_in_square(squares[0], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_1', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE2', look_in_square(squares[1], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_2', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE3', look_in_square(squares[2], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_3', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE4', look_in_square(squares[3], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_4', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE5', look_in_square(squares[4], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_5', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE6', look_in_square(squares[5], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_6', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE7', look_in_square(squares[6], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_7', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('SQUARE8', look_in_square(squares[7], cam_pixel_to_point))
        Sequence.add('SHORTCIRCUIT_8', ReturnFunctionState(found_shape, ['ok', 'found']), transitions={'found': 'ABSORB'})
        Sequence.add('ABSORB', AbsorbResultState())
    return sq


def look_in_square(square, cam_pixel_to_point):  # type: (ParkingSquare, CamPixelToPointServer) -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok', input_keys=['green_shape'])
    offset = 0.5
    v = 0.2

    dt = offset / v

    def goal():
        return square.pose

    with sq:
        Sequence.add('SHORTCIRCUIT', ShortCircuitParkingSquareState(square), transitions={'shortcircuit': 'ABSORB'})
        Sequence.add('PARK', park_into_pose(goal), transitions={'err': 'ABSORB'})
        Sequence.add('FIND', SearchForShapeInSquareState(square, cam_pixel_to_point))
        Sequence.add('MOVE_BACK', ForwardState(-v, dt))
        Sequence.add('ABSORB', AbsorbResultState())
    return sq


def push_cube(cube, marker):  # type: (ARCube, ARTag) -> StateMachine
    sq = Sequence(outcomes=['ok'], connector_outcome='ok')

    def distance():
        return np.linalg.norm(numpify(cube.pose.pose.position) - numpify(marker.pose.pose.position))

    def can_see_cube():
        return rospy.get_time() - cube.last_seen < 0.2

    with sq:
        Sequence.add('MOVE_BEHIND_CUBE', navigate_behind_cube(parking_square_target(marker, offset=0.4), cube), transitions={'err': 'ABSORB'})
        Sequence.add('PUSH', PushToGoalState(cube, parking_square_target(marker, offset=0.4), v=0.2))
        Sequence.add('BACK', ForwardState(-0.2))
        Sequence.add('REDO_BACK', ReturnFunctionState(lambda: 'ok' if can_see_cube() else 'back', ['ok', 'back']), transitions={'back': 'BACK'})
        Sequence.add('REDO', ReturnFunctionState(lambda: 'ok' if distance() < 0.5 else 'redo', ['ok', 'redo']), transitions={'redo': 'MOVE_BEHIND_CUBE'})
        Sequence.add('NOTIFY', FunctionState(lambda: notify_pushed()))
        Sequence.add('ABSORB', AbsorbResultState())
    return sq
