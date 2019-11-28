from smach import State

from src.feature_detector.scripts.feature_detector import FeatureDetector
from src.util.scripts.util import notify_count


class Location2State(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'], output_keys=['green_shape'])
        self.feature_detector = FeatureDetector()

    def execute(self, ud):
        try:
            features = self.feature_detector.get_features()
            green_shape = next(f.shape for f in features if f.colour == 'green')
            ud.green_shape = green_shape
            notify_count(len(features))
            print(green_shape)
            return 'ok'
        except Exception, e:
            print(e)
            ud.green_shape = None
            return 'err'
