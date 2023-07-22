from sim.model_parameters.parameters.parameter import Parameter


class ToggleParameter(Parameter):
    def __init__(self, val: bool = None):
        self.val = val

    def get(self):
        return self.val

    def is_set(self) -> bool:
        return self.val is not None
