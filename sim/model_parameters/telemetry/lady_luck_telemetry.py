from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.telemetry.telemetry import Telemetry


class LadyLuckTelemetry(Telemetry):
    def __init__(self):
        super().__init__()
        self._model_name = "Lady Luck Telemetry System"

        self.data_delay = ConstantParameter(0)
        self.accelerometer_rate = ConstantParameter(100)
        self.accelerometer_variance = ConstantParameter(0.1)
        self.accelerometer_precision = ConstantParameter(0.03)
        self.wheelspeed_rate = ConstantParameter(100)
        self.wheelspeed_variance = ConstantParameter(0.2)
        self.wheelspeed_precision = ConstantParameter(0.1)
