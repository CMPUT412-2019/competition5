import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def white_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    return cv2.inRange(hsv, np.array([0, 0, 200]), np.array([255, 70, 255])).astype(bool)


def red_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    f = (
            cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 255])) |
            cv2.inRange(hsv, np.asarray([160, 70, 50]), np.asarray([180, 255, 255]))
    ).astype(bool)
    # f[f.shape[0]-40:f.shape[0], :] = False
    return f


def lower_filter(hsv):    # type: (np.ndarray) -> np.ndarray
    f = np.zeros(hsv.shape[0:2], dtype=bool)
    f[f.shape[0]-50:f.shape[0], :] = True
    return f


def red_lower_filter(hsv):    # type: (np.ndarray) -> np.ndarray
    return red_filter(hsv) & lower_filter(hsv)


class Detector(object):
    def __init__(self, filter, threshold, gui_topic):
        self.gui_filter_pub = rospy.Publisher(gui_topic, Image, queue_size=1)
        self.cv_bridge = CvBridge()
        self.filter = filter
        self.threshold = threshold

    def __call__(self, hsv):   # type: (np.ndarray) -> np.ndarray
        I = self.filter(hsv)
        self.gui_filter_pub.publish(self.cv_bridge.cv2_to_imgmsg(I.astype('uint8') * 255))
        return np.sum(I) > self.threshold


class RedDetector(Detector):
    def __init__(self):
        super(RedDetector, self).__init__(red_filter, 3000, 'gui/red_filter')


class RedLowerDetector(Detector):
    def __init__(self):
        super(RedLowerDetector, self).__init__(red_lower_filter, 300, 'gui/red_filter')
