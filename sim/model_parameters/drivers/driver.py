from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters


class Driver(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Driver"
        self._model_name = "Unnamed Driver"

        self.acceleration_confidence = ConstantParameter()
        self.braking_confidence = ConstantParameter()
        self.cornering_confidence = ConstantParameter()

        self._lock()
