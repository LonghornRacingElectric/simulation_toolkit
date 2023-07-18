from sim.model_parameters.cars.car import Car
from sim.model_parameters.parameters.constant_parameter import ConstantParameter


class LadyLuck(Car):
    def __init__(self):
        super().__init__()
        self._model_name = "Lady Luck"

        self.motor_max_torque = ConstantParameter(100)
        self.wheel_radius = ConstantParameter(0.2)
        self.mass = ConstantParameter(250)
