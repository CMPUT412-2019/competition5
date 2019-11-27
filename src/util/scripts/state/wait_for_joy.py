import rospy
from smach import State

from src.util.scripts.joystick_input import JoystickInput


class WaitForJoyState(State):
    def __init__(self):
        super(WaitForJoyState, self).__init__(outcomes=['ok'])
        self.joystick = JoystickInput()

    def execute(self, ud):
        while not rospy.is_shutdown():
            button = self.joystick.wait_for_press()
            if button == 'A':
                return 'ok'
