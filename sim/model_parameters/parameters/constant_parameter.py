from sim.model_parameters.parameters.parameter import Parameter


class ConstantParameter(Parameter):
    def __init__(self, val: float = None):
        self.val = val

    def get(self):
        return self.val

    def is_set(self) -> bool:
        return self.val is not None
