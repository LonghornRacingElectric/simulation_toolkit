from abc import abstractmethod

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector


class VehicleSystemModel:

    """
    These lists tell the simulation orchestrator which inputs/outputs to pass in and out of the systems.
    Aka mux/demux handled automatically.
    """
    def __init__(self):
        self.controls_in: list[str] = []
        self.state_in: list[str] = []
        self.state_out: list[str] = []
        self.observables_out: list[str] = []

    """
    Note that input and output vectors are passed as parameters to the function. There is no return value.
    """
    @abstractmethod
    def eval(self, car: Car, controls_in: ControlsVector, state_in: StateVector,
             state_out: StateDotVector, observables_out: ObservablesVector):
        pass
