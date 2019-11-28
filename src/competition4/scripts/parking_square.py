import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from ros_numpy import msgify, numpify


class ParkingSquare:
    def __init__(self, pose_name):  # type: (str) -> None
        position = [float(x) for x in rospy.get_param('named_poses/{}/position'.format(pose_name))]
        orientation = [float(x) for x in rospy.get_param('named_poses/{}/orientation'.format(pose_name))]
        self._pose = PoseStamped()
        self._pose.header.frame_id = 'map'
        self._pose.pose.position = msgify(Point, position)
        self._pose.pose.orientation = msgify(Quaternion, orientation)
        self._contains = set()

    @property
    def pose(self):
        return self._pose

    def distance_to(self, position):  # type: (Point) -> float
        return np.sqrt((position.x - self.pose.pose.position.x)**2, (position.y-self.pose.pose.position.y)**2)

    def contains_marker(self):
        return 'marker' in self._contains

    def contains_cube(self):
        return 'cube' in self._contains

    def contains_shape(self):
        return 'shape' in self._contains

    def contains_wrong_shape(self):
        return 'wrong_shape' in self._contains

    def set_contains_marker(self, contains_marker=True):  # type: (bool) -> None
        self._set_contains('marker', contains_marker)

    def set_contains_cube(self, contains_cube=True):  # type: (bool) -> None
        self._set_contains('cube', contains_cube)

    def set_contains_shape(self, contains_shape=True):  # type: (bool) -> None
        self._set_contains('shape', contains_shape)

    def set_contains_wrong_shape(self, contains_wrong_shape=True):
        self._set_contains('wrong_shape', contains_wrong_shape)

    def _set_contains(self, type, contains):  # type: (str, bool) -> None
        if contains:
            self._contains.add(type)
        else:
            self._contains.remove(type)


def closest_square(position, squares):  # type: (Point, List[ParkingSquare]) -> ParkingSquare
    differences = [np.linalg.norm(numpify(position)[:2] - numpify(square.pose.pose.position)[:2])
                   for square in squares]
    return squares[np.argmin(differences)]
