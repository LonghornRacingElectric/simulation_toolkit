from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters


class Telemetry(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Telemetry System"
        self._model_name = "Unnamed Telemetry System"

        self.data_delay = ConstantParameter()
        self.accelerometer_rate = ConstantParameter()
        self.accelerometer_variance = ConstantParameter()
        self.wheelspeed_rate = ConstantParameter()
        self.wheelspeed_variance = ConstantParameter()
        self.wheelspeed_precision = ConstantParameter()

        self._lock()
