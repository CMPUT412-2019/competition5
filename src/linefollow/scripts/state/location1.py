from smach import State

from src.util.scripts.cam_pixel_to_point import CamPixelToPointServer
from src.feature_detector.scripts.feature_detector import filter_by_distance, FeatureDetector
from src.util.scripts.util import notify_count


class Location1State(State):
    def __init__(self, cam_pixel_to_point):  # type: (CamPixelToPointServer) -> None
        State.__init__(self, outcomes=['ok', 'err'])
        self.feature_detector = FeatureDetector()
        self.cam_pixel_to_point = cam_pixel_to_point

    def execute(self, ud):
        try:
            features = self.feature_detector.get_features()
            features = [f for f in features if f.colour == 'red']
            features = filter_by_distance(features, max_distance=1., cam_pixel_to_point=self.cam_pixel_to_point)
            notify_count(min(3, len(features)))
            return 'ok'
        except Exception, e:
            print(e)
            return 'err'
