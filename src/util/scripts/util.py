import time
from os import path
from threading import Thread

import numpy as np
import playsound
import rospy
from geometry_msgs.msg import PointStamped
from kobuki_msgs.msg import Led
from sensor_msgs.msg import LaserScan
from tf import TransformListener, LookupException, ExtrapolationException, ConnectivityException, transformations
# from typing import Any, Callable, Optional


class PublisherValue:
    def __init__(self, name, data_class, rate, value_fn, **kwargs):
        self.rate = rospy.Rate(rate)
        self.value_fn = value_fn
        self._publisher = rospy.Publisher(name, data_class, **kwargs)
        self._thread = Thread(target=self._spin)
        self._thread.start()

    def _spin(self):
        while not rospy.is_shutdown():
            value = self.value_fn()
            if value is not None:
                self._publisher.publish(value)
            self.rate.sleep()


class SubscriberValue:
    def __init__(self, name, data_class, wait=True, queue_size=1, transform=None):  # type: (str, Any, bool, int, Optional[Callable[[Any], Any]]) -> None
        self._topic = name
        self._wait = wait
        self._transform = transform
        self._value = None
        self._subscriber = rospy.Subscriber(name, data_class, callback=self._callback, queue_size=queue_size)

    def _callback(self, message):
        if self._transform is None:
            self._value = message
        else:
            self._value = self._transform(message)

    def wait(self):
        while self._value is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}...'.format(self._topic))
            rospy.sleep(0.1)
        return self._value

    @property
    def value(self):
        if self._wait:
            self.wait()
        return self._value


class ProximityDetector:
    def __init__(self, proximity=1):
        self.proximity = proximity
        self.laser_scan = SubscriberValue('scan', LaserScan)

    def __call__(self, _):
        return np.nanmin(self.laser_scan.value.ranges) < self.proximity


def led(msg):  # type: (str) -> None
    msg = msg.upper()

    led_pub = {1: rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1, latch=True),
               2: rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1, latch=True)}
    led_col = {'R': Led.RED, 'G': Led.GREEN, 'O': Led.ORANGE, 'B': Led.BLACK}

    i = 0
    while i < len(msg):
        if msg[i] in led_col.keys() and i + 1 < len(msg):
            led_pub[int(msg[i+1])].publish(led_col[msg[i]])
            i = i + 1
        elif msg[i] == 'W':
            time.sleep(2)
        else:
            raise ValueError('Invalid message character: {}'.format(msg[i]))
        i = i + 1


def notify_count(count):
    count = max(1, count)
    count = min(3, count)
    led({1: 'b1b2', 2: 'g1b2', 3: 'g1g2'}.get(count, 'b1b2'))
    playsound.playsound(path.join(path.dirname(__file__), '../sound/{}.mp3'.format(count)), block=True)
    led('b1b2')

def notify_location3_match():
    led('o1o2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/match.mp3'), block=True)
    led('b1b2')

def notify_location4_match():
    led('o1b2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/match.mp3'), block=True)
    led('o1g2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/parked.mp3'), block=True)
    led('b1b2')

def notify_artag():
    led('g1b2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/artag.mp3'), block=True)
    led('b1b2')

def notify_cube():
    led('r1b2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/artag.mp3'), block=True)
    led('b1b2')

def notify_pushed():
    led('g1r2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/pushed.mp3'), block=True)
    led('b1b2')

def notify_finished():
    led('g1g2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/finished.mp3'), block=True)
    led('b1b2')

def notify_unmarked():
    led('r1r2')
    playsound.playsound(path.join(path.dirname(__file__), '../sound/unmarked.mp3'), block=True)
    led('b1b2')


def notify_number(n):  # type:  (int) -> None
    playsound.playsound(path.join(path.dirname(__file__), '../sound/number_{}.mp3'.format(int(n))), block=True)


def angle_diff(a, b):
    diff = a - b
    if diff < -np.pi: diff += 2 * np.pi
    if diff > np.pi: diff -= 2 * np.pi
    return diff


def convert_frame(transform_listener, point, target_frame, timeout=None):  # type: (TransformListener, PointStamped, str, Optional[float]) -> PointStamped
    start_time = rospy.get_time()
    while not rospy.is_shutdown() and (timeout is None or rospy.get_time() - start_time < timeout):
        try:
            return transform_listener.transformPoint(target_frame, point)
        except (ConnectivityException, LookupException, ExtrapolationException):
            print('Error transforming {} -> {}'.format(point.header.frame_id, target_frame))


def normal_to_quaternion(normal):  # type: (np.ndarray) -> np.ndarray
    angle = np.arctan2(normal[1], normal[0])
    return transformations.quaternion_about_axis(angle, [0., 0., 1.])

