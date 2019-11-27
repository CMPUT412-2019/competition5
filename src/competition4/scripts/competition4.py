#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from smach import Sequence
from smach_ros import IntrospectionServer

from src.boxpush.scripts.state.boxpush import NavigateBehindCubeState, LineUpBehindCubeState, PushToGoalState
from src.competition4.scripts.state_groups import move_to_stop_line, location1, enter_split, move_to_obstacle, location2, \
    exit_split, off_ramp, ar_tag, joystick_location, on_ramp, location3
from src.util.scripts.ar_tag import ARTag, ARCube
from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.util.scripts.state.wait_for_joy import WaitForJoyState


class UserData:
    def __init__(self):
        self.green_shape = None


def main():
    rospy.init_node('competition2')

    marker = ARTag(1)
    cube = ARCube(1)
    cam_pixel_to_point = CamPixelToPointServer()

    def target():  # type: () -> PoseStamped
        target = PoseStamped()
        target.pose.orientation.w = 1.0
        target.pose.position = Point(1.0, 1.0, 0.0)
        return target

    sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with sq:
        # Sequence.add('START', WaitForJoyState())

        # Sequence.add('STOP1', move_to_stop_line())
        # Sequence.add('LOCATION1', location1(cam_pixel_to_point))
        #
        # Sequence.add('STOP2', move_to_stop_line())
        #
        # Sequence.add('SPLIT_ENTER', enter_split())
        #
        # Sequence.add('OBSTACLE1', move_to_obstacle())
        # Sequence.add('LOCATION2', location2())
        #
        # Sequence.add('SPLIT_EXIT', exit_split())
        #
        # Sequence.add('STOP3', move_to_stop_line())

        Sequence.add('MOVE_BEHIND_CUBE', NavigateBehindCubeState(target, cube, 0.5))
        Sequence.add('LINE_UP', LineUpBehindCubeState(cube, 0.2))
        Sequence.add('PUSH', PushToGoalState(target, 0.2))

        # Sequence.add('OFFRAMP', off_ramp())
        #
        # Sequence.add('ARTAG', ar_tag(marker))
        # Sequence.add('JOYLOC', joystick_location())
        #
        # Sequence.add('ONRAMP', on_ramp())
        #
        # Sequence.add('STOP4', move_to_stop_line(lower=True))
        #
        # Sequence.add('LOCATION3', location3(cam_pixel_to_point))
        #
        # Sequence.add('STOP5', move_to_stop_line())

    sis = IntrospectionServer('smach_server', sq, '/SM_ROOT')
    sis.start()
    print('Executing...')
    sq.execute()


if __name__ == '__main__':
    main()
