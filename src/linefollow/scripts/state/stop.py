import time

import rospy
from geometry_msgs.msg import Twist
from smach import State


class StopState(State):
    def __init__(self, delay_time=.5):
        State.__init__(self, outcomes=['ok'])
        self.delay_time = delay_time
        self.rate = rospy.Rate(10)
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, ud):
        start_time = time.time()
        while not rospy.is_shutdown() and time.time() - start_time < self.delay_time:
            self.twist_pub.publish(Twist())
            self.rate.sleep()

        return 'ok'
