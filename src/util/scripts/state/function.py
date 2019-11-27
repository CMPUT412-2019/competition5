from collections import Callable

from smach import State


class FunctionState(State):
    def __init__(self, function, outcome='ok'):  # type: (Callable, str) -> None
        super(FunctionState, self).__init__(outcomes=[outcome])
        self.outcome = outcome
        self.function = function

    def execute(self, ud):
        self.function()
        return self.outcome
