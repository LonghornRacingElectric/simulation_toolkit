from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector


# TODO this entire thing is temporary for testing, do not actually use

class PowertrainModel(VehicleSystemModel):
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

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_in_vector: StateVector,
             state_out_vector: StateDotVector, observables_out_vector: ObservablesVector):

        torque = controls_vector.torque_request  # monkey physics, pls disregard
        acceleration = torque / vehicle_parameters.wheel_radius / vehicle_parameters.mass

        state_out_vector.acceleration = acceleration
        observables_out_vector.torque = torque
