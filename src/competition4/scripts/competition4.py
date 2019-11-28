#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from smach import Sequence
from smach_ros import IntrospectionServer

from src.boxpush.scripts.state.boxpush import NavigateBehindCubeState, LineUpBehindCubeState, PushToGoalState, \
    parking_square_target
from src.competition4.scripts.parking_square import ParkingSquare
from src.competition4.scripts.state.search_for_markers import search_for_tags
from src.competition4.scripts.state_groups import move_to_stop_line, location1, enter_split, move_to_obstacle, \
    location2, \
    exit_split, off_ramp, ar_tag, joystick_location, on_ramp, location3, find_shape
from src.util.scripts.ar_tag import ARTag, ARCube
from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.util.scripts.state.wait_for_joy import WaitForJoyState
from src.util.scripts.util import notify_finished


class UserData:
    def __init__(self):
        self.green_shape = None


def main():
    rospy.init_node('competition2')

    wall_offset = 0.4

    marker = ARTag(30, visual_topic='/viz/marker')
    cube = ARCube(2, visual_topic='/viz/cube')
    cam_pixel_to_point = CamPixelToPointServer()
    squares = []
    for i in range(1, 9):
        squares.append(ParkingSquare('S{}'.format(i)))

    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sq:
        Sequence.add('START', WaitForJoyState())

        Sequence.add('STOP1', move_to_stop_line())
        Sequence.add('LOCATION1', location1(cam_pixel_to_point))

        Sequence.add('STOP2', move_to_stop_line())

        Sequence.add('SPLIT_ENTER', enter_split())

        Sequence.add('OBSTACLE1', move_to_obstacle())
        Sequence.add('LOCATION2', location2())

        Sequence.add('SPLIT_EXIT', exit_split())

        Sequence.add('STOP3', move_to_stop_line())

        Sequence.add('OFFRAMP', off_ramp())

        Sequence.add('SEARCH_FOR_MARKERS', search_for_tags(squares, marker, cube))

        Sequence.add('GO_TO_MARKER', ar_tag(marker))

        Sequence.add('MOVE_BEHIND_CUBE', NavigateBehindCubeState(parking_square_target(marker, wall_offset), cube, 0.7))
        Sequence.add('PUSH', PushToGoalState(cube, parking_square_target(marker, wall_offset), 0.2))

        Sequence.add('FIND_SHAPE', find_shape(squares, cam_pixel_to_point))

        Sequence.add('ONRAMP', on_ramp())

        Sequence.add('STOP4', move_to_stop_line(lower=True))

        Sequence.add('LOCATION3', location3(cam_pixel_to_point))

        Sequence.add('STOP5', move_to_stop_line())

    sis = IntrospectionServer('smach_server', sq, '/SM_ROOT')
    sis.start()
    print('Executing...')
    sq.execute()
    notify_finished()


if __name__ == '__main__':
    main()
