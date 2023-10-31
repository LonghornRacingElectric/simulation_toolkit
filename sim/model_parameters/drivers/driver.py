from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class Driver(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Driver"
        self._model_name = "Unnamed Driver"

        self.acceleration_confidence = ConstantParameter()
        self.braking_confidence = ConstantParameter()
        self.cornering_confidence = ConstantParameter()

        self._lock()

    def driver_program(self, time: float, state: StateVector, state_dot: StateDotVector) -> DriverControlsVector:
        raise Exception("This driver needs a driver_program()!")
