from __future__ import division, print_function

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from tf import TransformListener

from src.util.scripts.util import SubscriberValue, convert_frame


class CamPixelToPointServer:
    def __init__(self):
        self.camera_model = PinholeCameraModel()
        self.bridge = CvBridge()
        self.model_set = False
        self.tf_listener = TransformListener()

    def pixel_to_point(self, cam_pos, out_frame='map'):  # type: (np.ndarray) -> PointStamped
        if not self.model_set:
            self.camera_model.fromCameraInfo(rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo))
            self.model_set = True
        x, y = int(cam_pos[0]), int(cam_pos[1])
        d = self.read_depth_simple(x, y)
        pos = np.array(self.camera_model.projectPixelTo3dRay((x, y))) * d

        point = PointStamped()
        point.header.frame_id = self.camera_model.tfFrame()
        point.point.x, point.point.y, point.point.z = pos[0], pos[1], pos[2]
        point = convert_frame(self.tf_listener, point, out_frame)
        return point

    def read_depth_simple(self, x, y):  # (int, int) -> float
        image = rospy.wait_for_message('/camera/depth_registered/image_raw', Image)
        image = self.bridge.imgmsg_to_cv2(image)
        return image[y, x]
