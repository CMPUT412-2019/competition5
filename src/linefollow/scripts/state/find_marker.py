import rospy
from geometry_msgs.msg import Twist
from smach import State

from src.util.scripts.ar_tag import ARTag


class FindMarkerState(State):
    """
    Turns the rover in place until it sees the AR marker.
    """
    def __init__(self, marker, cmd_vel_topic):  # type: (ARTag, str) -> None
        State.__init__(self, outcomes=['ok'])
        self.marker = marker
        self.twist_publisher = rospy.Publisher(cmd_vel_topic, Twist)

    def execute(self, ud):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.marker.visible:
                return 'ok'
            t = Twist()
            t.angular.z = 0.3
            self.twist_publisher.publish(t)
            rate.sleep()
