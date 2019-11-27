import rospy
from smach import State

from src.util.scripts.joystick_input import JoystickInput
from src.util.scripts.util import notify_number


class SelectNumberState(State):
    def __init__(self, min, max):
        super(SelectNumberState, self).__init__(outcomes=['ok'], output_keys=['number'])
        self.number = min
        self.min = min
        self.max = max
        self.joystick = JoystickInput()

    def execute(self, ud):
        self.number = self.min
        while not rospy.is_shutdown():
            print('NUMBER IS {}'.format(self.number))
            notify_number(self.number)
            button = self.joystick.wait_for_press()
            if button == 'RB':
                self.number += 1
            elif button == 'LB':
                self.number -= 1
            elif button == 'A':
                ud.number = self.number
                return 'ok'
            if self.number < self.min:
                self.number = self.max
            elif self.number > self.max:
                self.number = self.min
