from sim.system_models.system_model import SystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_in_vector import ControlsInVector
from sim.system_models.vectors.observables_out_vector import ObservablesOutVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector


# TODO this entire thing is temporary for testing, do not actually use

class PowertrainModel(SystemModel):
    def __init__(self):
        super().__init__()

        self.controls_in = [
            "torque_request"
        ]

        self.state_in = []

        self.state_out = [
            "acceleration"
        ]

        self.observables_out = [
            "torque"
        ]

    def eval(self, vehicle_parameters: Car, controls_in_vector: ControlsInVector, state_in_vector: StateVector,
             state_out_vector: StateDotVector, observables_out_vector: ObservablesOutVector):

        torque = controls_in_vector.torque_request  # monkey physics, pls disregard
        acceleration = torque / vehicle_parameters.wheel_radius / vehicle_parameters.mass

        state_out_vector.acceleration = acceleration
        observables_out_vector.torque = torque
