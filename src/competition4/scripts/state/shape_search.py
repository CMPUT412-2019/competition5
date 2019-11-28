from smach import State

from src.feature_detector.scripts.feature_detector import FeatureDetector, filter_by_distance, feature_depths
from src.util.scripts.util import notify_match


class SearchForShapeInSquareState(State):
    def __init__(self, square, cam_pixel_to_point):  # type: (ParkingSquare, CamPixelToPointServer) -> None:
        State.__init__(self, outcomes=['ok'], input_keys=['green_shape'])
        self.feature_detector = FeatureDetector()
        self.cam_pixel_to_point = cam_pixel_to_point
        self.square = square

    def execute(self, ud):
        features = self.feature_detector.get_features()
        features = [f for f in features if f.colour == 'red']
        features = filter_by_distance(features, 1., self.cam_pixel_to_point)
        depths = feature_depths(features, self.cam_pixel_to_point)
        feature = next((f for i, f in enumerate(features) if depths[i] == min(depths)), None)

        if feature is None:
            return 'ok'

        print(feature.shape)

        if feature.shape == ud.green_shape:
            notify_match()
            self.square.set_contains_shape()
            return 'match'

        self.square.set_contains_wrong_shape()
        return 'ok'


class ShortCircuitParkingSquareState(State):
    def __init__(self, square):  # type: (ParkingSquare) -> None
        State.__init__(self, outcomes=['ok', 'shortcircuit'])
        self.square = square

    def execute(self, ud):
        if self.square.contains_marker() or self.square.contains_cube() or self.square.contains_wrong_shape():
            return 'shortcircuit'
        else:
            return 'ok'


