from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector


# TODO robert pls implement <3

class AeroModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        self.controls_in = [

        ]

        self.state_in = [

        ]

        self.state_out = [

        ]

        self.observables_out = [

        ]

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_in_vector: StateVector,
             state_out_vector: StateDotVector, observables_out_vector: ObservablesVector):
        pass
