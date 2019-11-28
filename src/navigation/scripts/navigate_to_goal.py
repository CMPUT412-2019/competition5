from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from smach import State

from src.navigation.scripts.move_to import move_to


class NavigateToGoalState(State):
    """
    Moves the robot to goal in the map frame.
    """
    def __init__(self, goal):  # type: (Callable[[], PoseStamped]) -> None
        State.__init__(self, outcomes=['ok', 'err'])
        self.goal = goal
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        pose = self.goal()
        if pose is None:
            return 'err'
        return 'ok' if move_to(self.client, pose) else 'err'
