# TODO implement
from abc import abstractmethod

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_in_vector import ControlsInVector
from sim.system_models.vectors.observables_out_vector import ObservablesOutVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector


class SystemModel:
    """
    These lists tell the simulation orchestrator which inputs/outputs to pass in and out of the systems.
    Aka mux/demux handled automatically.
    """
    def __init__(self):
        self.controls_in: list[str] = []
        self.state_in: list[str] = []
        self.state_out: list[str] = []
        self.observables_out: list[str] = []

    @abstractmethod
    def eval(self, vehicle_parameters: Car, controls_in_vector: ControlsInVector, state_in_vector: StateVector,
             state_out_vector: StateDotVector, observables_out_vector: ObservablesOutVector):
        pass
