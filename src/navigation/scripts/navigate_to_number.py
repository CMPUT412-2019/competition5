from smach import State

from src.navigation.scripts.navigate_to_named_pose import NavigateToNamedPoseState


class NavigateToNumberState(State):
    """
    Moves the robot to the parking square specified by a number

    Required userdata
      - number (int): The number of the square at which to park
    """
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['number'])

    def execute(self, ud):
        return NavigateToNamedPoseState('S{}'.format(ud.number)).execute({})
