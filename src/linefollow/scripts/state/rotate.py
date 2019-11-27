import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist
from smach import State


class RotateState(State):
    def __init__(self, angle=np.pi/2):
        State.__init__(self, outcomes=['ok'])
        self.angle = angle
        self.rate = rospy.Rate(1000)

        self.pose2d = None  # type: Pose2D

        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('pose2d', Pose2D, callback=self.pose2d_callback)

    def pose2d_callback(self, msg):
        self.pose2d = msg

    def execute(self, ud):
        while not rospy.is_shutdown() and self.pose2d is None:
            self.rate.sleep()

        angle_target = self.pose2d.theta + self.angle
        if angle_target > np.pi:
            angle_target -= 2*np.pi
        elif angle_target < -np.pi:
            angle_target += 2*np.pi

        while not rospy.is_shutdown():
            err = angle_target - self.pose2d.theta
            if abs(err) < 0.1:
                break

            t = Twist()
            t.angular.z = 2 * np.sign(err) * min(2, abs(err))
            self.twist_pub.publish(t)
            self.rate.sleep()

        return 'ok'
