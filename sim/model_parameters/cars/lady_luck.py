from sim.model_parameters.cars.car import Car
from sim.model_parameters.parameters import *


class LadyLuck(Car):
    def __init__(self):
        super().__init__()
        self._model_name = "Lady Luck"
