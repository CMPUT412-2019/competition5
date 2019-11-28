import numpy as np
import rospy
from geometry_msgs.msg import Twist
from smach import State, StateMachine

from src.util.scripts.parking_square import ParkingSquare, closest_square
from src.navigation.scripts.navigate_to_named_pose import NavigateToNamedPoseState
from src.util.scripts.ar_tag import ARTag, ARCube
from src.util.scripts.state.absorb_result import AbsorbResultState


class FindTags(State):
    def __init__(self, squares, marker, cube, rotate_speed=0.5):  # type: (List[ParkingSquare], ARTag, ARCube, float) -> None
        super(FindTags, self).__init__(outcomes=['ok', 'found'])
        self.marker = marker
        self.cube = cube
        self.squares = squares
        self.rotate_speed = rotate_speed
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    def execute(self, ud):
        rate = rospy.Rate(10)
        duration = 2*np.pi/self.rotate_speed * 1.5
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time()-start_time < duration:
            if self.marker.pose is not None:
                closest_square(self.marker.pose.pose.position, self.squares).set_contains_marker()
            if self.cube.pose is not None:
                closest_square(self.cube.pose.pose.position, self.squares).set_contains_cube()

            if self.marker.pose is not None and self.cube.pose is not None:
                return 'found'
            t = Twist()
            t.angular.z = self.rotate_speed
            self.twist_pub.publish(t)
            rate.sleep()
        return 'ok'


def search_for_tags(squares, marker, cube):  # type: (List[ParkingSquare], ARTag, ARCube) -> StateMachine
    sm = StateMachine(outcomes=['ok'])
    with sm:
        StateMachine.add('GO_TO_AR_SEARCH_1', NavigateToNamedPoseState('ar_search_1'), transitions={'ok': 'SEARCH1'})
        StateMachine.add('SEARCH1', FindTags(squares, marker, cube), transitions={'ok': 'GO_TO_AR_SEARCH_2', 'found': 'ABSORB'})
        StateMachine.add('GO_TO_AR_SEARCH_2', NavigateToNamedPoseState('ar_search_2'), transitions={'ok': 'SEARCH2'})
        StateMachine.add('SEARCH2', FindTags(squares, marker, cube), transitions={'ok': 'GO_TO_AR_SEARCH_1', 'found': 'ABSORB'})
        StateMachine.add('ABSORB', AbsorbResultState())
    return sm
