from __future__ import print_function

import json
import numpy as np
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from typing import Tuple, List
from more_itertools import flatten

from src.util.scripts.util import SubscriberValue


class Feature:
    def __init__(self, shape, colour, centroid, bb):  # type: (str, str, np.ndarray, np.ndarray) -> None
        self.shape = shape
        self.colour = colour
        self.centroid = centroid
        self.bb = bb


class PredictClient:
    def __init__(self):
        self.cv_bridge = cv_bridge.CvBridge()
        self.request_publisher = rospy.Publisher('/pred/request', Image)
        self.result_subscriber = rospy.Subscriber('/pred/result', String, self._on_pred)
        self.result_publisher = rospy.Publisher('/pred/result_image', Image, queue_size=1)
        self.colours_by_name = {'red': (0, 0, 255), 'green': (0, 255, 0)}
        self.done = False
        self.prediction = None

    def predict(self, image):  # type: (Image) -> Tuple[List, List, List]
        rate = rospy.Rate(100)
        self.prediction = None
        self.request_publisher.publish(image)
        last_sent = rospy.get_time()
        while not rospy.is_shutdown() and self.prediction is None:
            if rospy.get_time() - last_sent > 1:
                last_sent = rospy.get_time()
                self.request_publisher.publish(image)
            rate.sleep()
        self._show_prediction(image, self.prediction)
        return self.prediction

    def _show_prediction(self, image, prediction):
        image = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(image), cv2.COLOR_BGR2RGB)
        boxes, labels, colours = prediction
        for (x, y, w, h), label, c in zip(boxes, labels, colours):
            cv2.rectangle(image, (x, y), (x + w, y + h), self.colours_by_name[c])
            cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.colours_by_name[c])
        self.result_publisher.publish(self.cv_bridge.cv2_to_imgmsg(image))

    def _on_pred(self, pred):
        self.prediction = json.loads(pred.data)


class FeatureDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.colours_by_name = {'red': (0, 0, 255), 'green': (0, 255, 0)}
        self.viz_image_pub = rospy.Publisher('/viz/feature_detector', Image, queue_size=1)

        self.predict_client = PredictClient()

    def get_features(self):
        duration = 2
        rate = rospy.Rate(100)
        start_time = rospy.get_time()
        boxes, labels, colours = [], [], []
        while not rospy.is_shutdown() and rospy.get_time() - start_time < duration:
            bb, l, c = self.predict_client.predict(rospy.wait_for_message('/camera/rgb/image_raw', Image))
            boxes.append(bb)
            labels.append(l)
            colours.append(c)
            rate.sleep()

        boxes, labels, colours = self._cluster_similar_features(boxes, labels, colours)

        return [Feature(l, c, self._bb_centroid(bb), bb)
                for l, c, bb in zip(labels, colours, boxes)]

    def _bb_centroid(self, bb):
        return np.array([bb[0] + bb[2]/2, bb[1] + bb[3]/2])

    def _cluster_similar_features(self, boxes, labels, colours):
        return boxes[0], labels[0], colours[0]

        # centroids = np.array([self._bb_centroid(bb) for bb in boxes[0]])
        #
        # centroids_flattened = np.array([self._bb_centroid(bb) for bb in flatten(boxes)])
        # boxes_flattened = np.array(list(flatten(labels)))
        # labels_flattened = np.array(list(flatten(labels)))
        # colours_flattened = np.array(list(flatten(colours)))
        # kmeans = KMeans(len(boxes[0])).fit(centroids_flattened)
        # idx_flattened = kmeans.labels_
        #
        # return [Feature(np.median(labels_flattened[i == idx_flattened])[0],
        #                 np.median(colours_flattened[i == idx_flattened])[0],
        #                 centroids_flattened[i[0]],
        #                 boxes_flattened[[i[0]]])
        #         for i in np.unique(idx_flattened)]


def feature_depths(features, cam_pixel_to_point):  # type: (List[Feature], CamPixelToPointServer) -> List[float]
    depths = []
    for feature in features:
        point = cam_pixel_to_point.pixel_to_point(feature.centroid, 'camera_link')
        depths.append(np.linalg.norm([point.point.x, point.point.y, point.point.z]))
    return depths


def filter_by_distance(features, max_distance, cam_pixel_to_point, remove_nan=True):  # type: (List[Feature], float, CamPixelToPointServer) -> List[Feature]
    filtered = []
    for feature in features:
        point = cam_pixel_to_point.pixel_to_point(feature.centroid, 'camera_link')
        if point.point.x <= max_distance:
            filtered.append(feature)
        elif np.isnan(point.point.x) and not remove_nan:
            filtered.append(feature)
    return filtered


def select_center(features, cam_pixel_to_point):  # type: (List[Feature], CamPixelToPointServer) -> Feature
    angles = []
    center_ray = np.array([1.0, 0.0, 0.0])
    for feature in features:
        point = cam_pixel_to_point.pixel_to_point(feature.centroid, 'camera_link')
        ray = np.array([point.point.x, point.point.y, point.point.z])
        ray /= np.linalg.norm(ray)
        angle = np.arccos(np.dot(center_ray, ray))
        angles.append(abs(angle))
    return features[np.argmin(angles)]


if __name__ == '__main__':
    rospy.init_node('feature_detector')
    fd = FeatureDetector()

    while not rospy.is_shutdown():
        raw_input('Press enter to make prediction')
        for f in fd.get_features():
            print(f.colour, f.shape, f.centroid, f.bb)