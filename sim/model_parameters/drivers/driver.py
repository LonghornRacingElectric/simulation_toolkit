from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters


class Driver(ModelParameters):
    def __init__(self):
        super().__init__()
        self.model_type = "Driver"
        self.model_name = "Unnamed Driver"
        self._parameters = {
            "acceleration_confidence": type(ConstantParameter()),
            "braking_confidence": type(ConstantParameter()),
            "cornering_confidence": type(ConstantParameter())
        }

