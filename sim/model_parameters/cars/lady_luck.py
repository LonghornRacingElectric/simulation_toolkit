from sim.model_parameters.cars.car import Car
from sim.model_parameters.parameters.constant_parameter import ConstantParameter


class LadyLuck(Car):
    def __init__(self):
        super().__init__()
        self.model_name = "Lady Luck"
        self.max_acceleration = ConstantParameter(10)

        # self._validate_parameters()
