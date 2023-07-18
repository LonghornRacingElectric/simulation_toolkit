from sim.model_parameters.parameters.parameter import Parameter


class ToggleParameter(Parameter):
    def __init__(self, val: bool = False):
        self.val = val

    def get(self):
        return self.val
