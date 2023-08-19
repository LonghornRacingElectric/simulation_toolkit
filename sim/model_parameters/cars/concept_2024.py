from sim.model_parameters.cars.car import Car
from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.model_parameters.parameters import ConstantParameter, CurveParameter, ToggleParameter


class Concept2024(LadyLuck):
    def __init__(self):
        super().__init__()
        self._model_name = "Concept 2024"


