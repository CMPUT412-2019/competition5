import time

import rospy
from geometry_msgs.msg import Twist
from smach import State


class ForwardState(State):
    def __init__(self, speed, delay_time=.5):
        State.__init__(self, outcomes=['ok'])
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.delay_time = delay_time
        self.speed = speed
        self.rate = rospy.Rate(10)

    def execute(self, ud):
        start_time = time.time()
        while not rospy.is_shutdown() and time.time() - start_time < self.delay_time:
            t = Twist()
            t.linear.x = self.speed
            self.twist_pub.publish(t)
            self.rate.sleep()

        return 'ok'
