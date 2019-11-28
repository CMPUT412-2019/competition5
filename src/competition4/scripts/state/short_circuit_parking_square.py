from smach import State


class ShortCircuitParkingSquareState(State):
    def __init__(self, square):  # type: (ParkingSquare) -> None
        State.__init__(self, outcomes=['ok', 'shortcircuit'])
        self.square = square

    def execute(self, ud):
        if self.square.contains_marker() or self.square.contains_cube() or self.square.contains_wrong_shape():
            return 'shortcircuit'
        else:
            return 'ok'