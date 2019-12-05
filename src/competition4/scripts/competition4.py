#!/usr/bin/env python

import rospy
from smach import Sequence
from smach_ros import IntrospectionServer

from src.competition4.scripts.state.search_for_markers import search_for_tags
from src.competition4.scripts.state_groups import move_to_stop_line, location1, enter_split, move_to_obstacle, \
    location2, exit_split, off_ramp, go_to_marker, on_ramp, location3, find_shape, push_cube
from src.linefollow.scripts.state.forward import ForwardState
from src.util.scripts.ar_tag import ARTag, ARCube
from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.util.scripts.parking_square import ParkingSquare
from src.util.scripts.state.function import FunctionState, ReturnFunctionState
from src.util.scripts.state.wait_for_joy import WaitForJoyState
from src.util.scripts.util import notify_finished


# Note: bottom camera settings:
#  * the lower field-of-view is 20 cm from the robot's bumper

class UserData:
    def __init__(self):
        self.green_shape = None


def main():
    rospy.init_node('competition2')

    wall_offset = 0.4

    marker = ARTag(20, visual_topic='/viz/marker')
    cube = ARCube(2, visual_topic='/viz/cube')
    cam_pixel_to_point = CamPixelToPointServer()
    squares = []
    for i in range(1, 9):
        squares.append(ParkingSquare(i))

    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sq:

        # Sequence.add('START', WaitForJoyState())

        Sequence.add('STOP1', move_to_stop_line())
        Sequence.add('LOCATION1', location1(cam_pixel_to_point))

        Sequence.add('STOP2', move_to_stop_line())

        Sequence.add('SPLIT_ENTER', enter_split())

        Sequence.add('OBSTACLE1', move_to_obstacle())
        Sequence.add('MOVEFORWARD', ForwardState(0.2, 1))
        Sequence.add('LOCATION2', location2())

        Sequence.add('SPLIT_EXIT', exit_split())

        Sequence.add('STOP3', move_to_stop_line())

        Sequence.add('OFFRAMP', off_ramp())
        #
        Sequence.add('SEARCH_FOR_MARKERS', search_for_tags(squares, marker, cube))

        # Sequence.add('GO_TO_MARKER', go_to_marker(marker, squares))

        Sequence.add('PUSH_CUBE', push_cube(cube, marker, squares))

        Sequence.add('FIND_SHAPE', find_shape(squares, cam_pixel_to_point))
        #
        Sequence.add('ONRAMP', on_ramp())

        Sequence.add('STOP4', move_to_stop_line(lower=True))

        Sequence.add('LOCATION3', location3(cam_pixel_to_point))

        Sequence.add('STOP5', move_to_stop_line())

        Sequence.add('NOTIFY', FunctionState(lambda: notify_finished()))
        Sequence.add('RESET', FunctionState(lambda: marker.reset()))
        Sequence.add('REDO', ReturnFunctionState(lambda: 'err', ['ok', 'err']), transitions={'err': 'STOP1'})

    # sq.userdata.green_shape = 'square'

    sis = IntrospectionServer('smach_server', sq, '/SM_ROOT')
    sis.start()
    print('Executing...')
    sq.execute()
    # try:
    #     notify_finished()
    # except Exception as e:
    #     print(e)


if __name__ == '__main__':
    main()
