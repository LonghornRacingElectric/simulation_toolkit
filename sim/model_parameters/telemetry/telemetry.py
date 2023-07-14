from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters


class Telemetry(ModelParameters):
    def __init__(self):
        super().__init__()
        self.model_type = "Telemetry System"
        self.model_name = "Unnamed Telemetry System"
        self._parameters = {
            "data_delay": type(ConstantParameter()),
            "accelerometer_rate": type(ConstantParameter()),
            "accelerometer_variance": type(ConstantParameter()),
            "accelerometer_precision": type(ConstantParameter()),
            "wheelspeed_rate": type(ConstantParameter()),
            "wheelspeed_variance": type(ConstantParameter()),
            "wheelspeed_precision": type(ConstantParameter()),
        }
