import time

import cv2
import cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from smach import State


class TransitionBehaviour:
    def __init__(self):
        pass

    def init(self):
        pass

    def tick(self, hsv):
        pass


class TransitionAfter(TransitionBehaviour):
    def __init__(self, func):
        TransitionBehaviour.__init__(self)

        self.func = func
        self.last_seen = None

    def init(self):
        self.last_seen = None

    def tick(self, hsv):
        seen = self.func(hsv)

        if seen:
            self.last_seen = time.time()
            return False
        elif self.last_seen is not None and time.time() - self.last_seen > 0.1:
            return True


class TransitionAt(TransitionBehaviour):
    def __init__(self, func):
        TransitionBehaviour.__init__(self)

        self.func = func

    def tick(self, hsv):
        if self.func(hsv):
            return True
        else:
            return False


class LineFollowState(State):
    class StopError(Exception):
        pass

    def __init__(self, forward_speed, angle_controller, line_filter, transition_behaviour):
        State.__init__(self, outcomes=['ok'])

        self.forward_speed = forward_speed
        self.angle_controller = angle_controller
        self.line_filter = line_filter

        self.transition_behaviour = transition_behaviour  # type: TransitionBehaviour

        self.bridge = cv_bridge.CvBridge()
        self.twist = Twist()
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.gui_line_pub = rospy.Publisher('gui/line_mask', Image, queue_size=1)

        self.rate = rospy.Rate(10)
        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def wait_for_image(self):
        while not rospy.is_shutdown() and self.image is None:
            self.rate.sleep()

    def execute(self, ud):
        image_sub = rospy.Subscriber('bottom_camera/image_raw', Image, self.image_callback)

        self.wait_for_image()
        self.transition_behaviour.init()

        try:
            while not rospy.is_shutdown():
                self.tick()
                self.rate.sleep()
        except self.StopError:
            return 'ok'
        finally:
            image_sub.unregister()

    def tick(self):
        image = np.copy(self.image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        eye_mask = self.get_eye_mask(hsv)
        line_mask = self.line_filter(hsv) & eye_mask

        # Check for stopping condition
        if self.transition_behaviour.tick(hsv):
            raise self.StopError()

        # Line following
        if not self.is_mask_empty(line_mask):
            cx, cy = self.get_mask_center(line_mask)
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = (cx - image.shape[1] / 2) / float(image.shape[1])

            t = Twist()
            t.linear.x = self.forward_speed * max((1. - abs(err) * 1.9), 0)
            t.angular.z = self.angle_controller.get(err)
            self.twist_pub.publish(t)

        # Display
        self.gui_line_pub.publish(self.bridge.cv2_to_imgmsg(line_mask * 255))

    @staticmethod
    def get_eye_mask(image):
        return np.ones(image.shape[:2]).astype('uint8')

    @staticmethod
    def get_mask_center(mask):
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy

    @staticmethod
    def is_mask_empty(mask):
        return np.all(mask == 0)