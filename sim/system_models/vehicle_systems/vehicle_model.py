from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.powertrain_model import PowertrainModel
from sim.system_models.vehicle_systems.suspension_model import SuspensionModel
from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel


class VehicleModel:
    def __init__(self, vehicle_parameters: Car):
        self.vehicle_parameters = vehicle_parameters
        self.vehicle_system_models: list[VehicleSystemModel] = [
            PowertrainModel(),
            SuspensionModel()
        ]

    def eval(self, controls: ControlsVector, state: StateVector) -> StateDotVector:
        state_dot: StateDotVector = StateDotVector()
        observables: ObservablesVector = ObservablesVector()

        for vehicle_system_model in self.vehicle_system_models:
            controls.expect_get(vehicle_system_model.controls_in)
            state.expect_get(vehicle_system_model.state_in)
            state_dot.expect_set(vehicle_system_model.state_out)
            observables.expect_set(vehicle_system_model.observables_out)

            vehicle_system_model.eval(self.vehicle_parameters, controls, state, state_dot, observables)

            controls.confirm_expectations()
            state.confirm_expectations()
            state_dot.confirm_expectations()
            observables.confirm_expectations()

        # TODO log state and observables!

        return state_dot
