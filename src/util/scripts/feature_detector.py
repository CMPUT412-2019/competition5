import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
# from typing import Any, List, Tuple

from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.util.scripts.util import SubscriberValue


class Feature:
    def __init__(self, shape, colour, centroid, contour):  # type: (str, str, np.ndarray, np.ndarray) -> None
        self.shape = shape
        self.colour = colour
        self.centroid = centroid
        self.contour = contour


def make_tracker(tracker_type):  # type: (str) -> Any
    constructors = {
        'Boosting': cv2.TrackerBoosting_create(),
        'MIL': cv2.TrackerMIL_create(),
        'KCF': cv2.TrackerKCF_create(),
        'TLD': cv2.TrackerTLD_create(),
        'MedianFlow': cv2.TrackerMedianFlow_create(),
        'GOTURN': cv2.TrackerGOTURN_create(),
        'MOSSE': cv2.TrackerMOSSE_create(),
        # 'CSRT': cv2.TrackerCSRT_create(),
    }
    return constructors[tracker_type]


class FeatureDetector:
    def convert_image(self, msg):
        # return cv2.medianBlur(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), 5)
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.normalize(image.astype(float), image)
        return image.astype(np.uint8)

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image = SubscriberValue('/camera/rgb/image_raw', Image, transform=self.convert_image)
        self.masks = {
            'red': self._red_mask,
            'green': self._green_mask,
        }
        self.gui_image_pub = rospy.Publisher('/gui/feature_detector', Image, queue_size=1)

    def get_features(self):  # type: () -> List[Feature]
        _ = self.image.value
        rospy.sleep(2)

        trackers = {col: cv2.MultiTracker_create() for col in self.masks.keys()}
        # Wait for image to warm up
        image = self.image.wait_for_n_messages(10)
        features = {}

        for col, mask in self._split_image_into_masks(image).items():
            boxes = self._find_boxes_to_track(mask)
            track_image = cv2.cvtColor(mask.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
            # print('ADDING BOXES: ', boxes)
            for box in boxes:
                trackers[col].add(make_tracker('Boosting'), track_image, box)
            # print(boxes)
            features[col] = [[] for _ in boxes]

        duration = 2
        rate = rospy.Rate(100)
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start_time < duration:
            image = self.image.value
            show_image = image
            for col, mask in self._split_image_into_masks(image).items():
                try:
                    track_image = cv2.cvtColor(mask.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
                    success, boxes = trackers[col].update(track_image)

                    boxes = [(box[0] - 10, box[1] - 10, box[2] + 20, box[3] + 20) for box in boxes]

                    # print(success, boxes)
                    for box in boxes:
                        x, y, w, h = (int(v) for v in box)
                        show_image = cv2.rectangle(show_image, (x, y), (x+w, y+h), (255, 0, 0), 5)

                    features_found = self._find_features(mask, boxes, col)
                    for index, feature in enumerate(features_found):
                        features[col][index].append(feature)

                    for feature in features_found:
                        if feature is not None:
                            show_image = cv2.drawContours(show_image, [feature.contour], -1, (255, 255, 0), 5)

                            col = self.col_name_to_rgb(feature.colour)
                            # print(col)
                            show_image = cv2.putText(
                                show_image,
                                '{}'.format(feature.shape),
                                tuple(int(x) for x in feature.centroid),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                3.0,
                                col,
                            )
                except Exception as err:
                    print(err)
            self.gui_image_pub.publish(self.bridge.cv2_to_imgmsg(show_image))

            rate.sleep()

        # cv2.waitKey(0)
        # print(features)
        features = sum(([self._combine_features(f) for f in features[col]] for col in self.masks.keys()), [])
        return [f for f in features if f is not None]


    @staticmethod
    def _combine_features(features):  # type: (List[Feature]) -> Feature
        features = [f for f in features if f is not None]
        if not features:
            return None
        shapes = [f.shape for f in features]
        cols = [f.colour for f in features]
        re = features[0]
        # https://stackoverflow.com/a/1518632
        re.shape = max(set(shapes), key=shapes.count)
        # print('*****************************')
        # print(float(len([s for s in shapes if s == 'circle']))/float(len(shapes)))
        # if re.shape == 'square' and float(len([s for s in shapes if s == 'circle']))/float(len(shapes)) > .20:
        #     re.shape = 'circle'
        re.colour = max(set(cols), key=cols.count)
        return re

    def _split_image_into_masks(self, image):  # type: (np.ndarray) -> Dict[str, np.ndarray]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return {col: mask_fn(hsv) for col, mask_fn in self.masks.items()}

    def _find_boxes_to_track(self, mask):  # type: () -> List[Tuple[float]]
        _, contours, _ = cv2.findContours(
            mask.astype(np.uint8) * 255,
            mode=cv2.RETR_EXTERNAL,
            method=cv2.CHAIN_APPROX_SIMPLE,
        )
        boxes = []
        for contour in contours:
            if cv2.contourArea(contour) < 100.0:
                continue
            boxes.append(cv2.boundingRect(contour))
        return boxes

    def _find_features(self, mask, boxes, col):  # type: (np.ndarray, List[Tuple[float]], str) -> List[Feature]
        features = []

        for box in boxes:
            x, y, w, h = (int(v) for v in box)
            box_mask = np.zeros(shape=mask.shape[:2], dtype=bool)
            box_mask[y:y+h, x:x+w] = True
            masked_image = mask & box_mask
            contour = self._get_largest_contour(masked_image)
            if contour is None:
                features.append(None)
                continue
            shape, contour = self._get_shape(contour, give_simplified_contour=True)
            if shape is None:
                features.append(None)
                continue
            centroid = self._get_centroid(contour)
            features.append(Feature(shape, col, centroid, contour))
        return features

    @staticmethod
    def _get_largest_contour(mask):  # type: (np.ndarray) -> np.ndarray
        _, contours, _ = cv2.findContours(
            mask.astype(np.uint8),
            mode=cv2.RETR_EXTERNAL,
            method=cv2.CHAIN_APPROX_SIMPLE,
        )
        areas = [
            cv2.contourArea(c) for c in contours
        ]
        try:
            return contours[np.argmax(areas)]
        except ValueError:
            return None

    @staticmethod
    def _get_centroid(contour):
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return contour[0]
        return np.array([
            M['m10']/M['m00'],
            M['m01']/M['m00'],
        ])

    @staticmethod
    def _bounding_quad(contour):
        errs = np.arange(0, 1, 0.005) * cv2.arcLength(contour, True)
        lo, hi = 0, len(errs)
        while lo <= hi:
            mid = (lo + hi) // 2
            contour_simplified = cv2.approxPolyDP(contour, errs[mid], closed=True)
            if len(contour_simplified) == 4:
                return contour_simplified.reshape((-2, 2))
            elif len(contour_simplified) < 4:
                hi = mid
            elif len(contour_simplified) > 4:
                lo = mid + 1
        return np.zeros((4, 2))

    @staticmethod
    def _quad_area(pts):
        b1 = np.linalg.norm(pts[0] - pts[1])
        h1 = np.sqrt(np.linalg.norm(pts[1] - pts[3]) ** 2 - np.linalg.norm(pts[0] - pts[1]) ** 2)
        b2 = np.linalg.norm(pts[2] - pts[3])
        h2 = np.sqrt(np.linalg.norm(pts[1] - pts[3]) ** 2 - np.linalg.norm(pts[2] - pts[3]) ** 2)
        return b1 * h1 / 2 + b2 * h2 / 2

    @staticmethod
    def _get_shape(contour, give_simplified_contour=False):
        contour_simple = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

        if len(contour_simple) < 3:
            shape = None
        elif len(contour_simple) == 3:
            contour = contour_simple
            shape = 'triangle'
        else:
            _, (a, b), _ = cv2.fitEllipse(contour)
            ellipse_area = np.pi * a * b / 4
            # quad_area = FeatureDetector._quad_area(FeatureDetector._bounding_quad(contour))
            _, (w, h), _ = cv2.minAreaRect(contour)
            rect_area = w * h

            shape = 'circle' if ellipse_area < rect_area else 'square'

        return (shape, contour) if give_simplified_contour else shape

    @staticmethod
    def _red_mask(hsv):  # type: (np.ndarray) -> np.ndarray
        mask_low = cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 255])) > 0.0
        mask_high = cv2.inRange(hsv, np.asarray([170, 70, 50]), np.asarray([180, 255, 255])) > 0.0
        return mask_low | mask_high

    @staticmethod
    def _green_mask(hsv):  # type: (np.ndarray) -> np.ndarray
        return cv2.inRange(hsv, np.asarray([40, 70, 50]), np.asarray([90, 255, 255])) > 0.0

    @staticmethod
    def col_name_to_rgb(colour):  # type: (str) -> Tuple[int, int, int]
        return {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
        }[colour]


def feature_depths(features, cam_pixel_to_point):  # type: (List[Feature], CamPixelToPointServer) -> List[float]
    depths = []
    for feature in features:
        point = cam_pixel_to_point.pixel_to_point(feature.centroid, 'camera_link')
        depths.append(np.linalg.norm([point.point.x, point.point.y, point.point.z]))
    return depths


def filter_by_distance(features, max_distance, cam_pixel_to_point):  # type: (List[Feature], float, CamPixelToPointServer) -> List[Feature]
    filtered = []
    for feature in features:
        point = cam_pixel_to_point.pixel_to_point(feature.centroid, 'camera_link')
        if point.point.x <= max_distance:
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
