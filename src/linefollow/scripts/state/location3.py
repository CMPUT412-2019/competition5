from smach import State

from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.feature_detector.scripts.feature_detector import feature_depths, filter_by_distance, FeatureDetector
from src.util.scripts.util import notify_location3_match


class Location3State(State):
    def __init__(self, cam_pixel_to_point):  # type: (CamPixelToPointServer) -> None
        State.__init__(self, outcomes=['ok', 'match', 'err'], input_keys=['green_shape'])
        self.feature_detector = FeatureDetector()
        self.cam_pixel_to_point = cam_pixel_to_point

    def execute(self, ud):
        try:
            features = self.feature_detector.get_features()
            features = [f for f in features if f.colour == 'red']
            features = filter_by_distance(features, 1., self.cam_pixel_to_point)
            depths = feature_depths(features, self.cam_pixel_to_point)
            feature = next((f for i, f in enumerate(features) if depths[i] == min(depths)), None)

            if feature is None:
                return 'err'

            print(feature.shape)

            if feature.shape == ud.green_shape:
                notify_location3_match()
                return 'match'

            return 'ok'
        except Exception, e:
            print(e)
            return 'err'
