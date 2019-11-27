from smach import State


class AbsorbResultState(State):
    def __init__(self, outcome='ok'):  # type: (str) -> None
        super(AbsorbResultState, self).__init__(outcomes=[outcome])
        self.outcome = outcome

    def execute(self, ud):
        return self.outcome
