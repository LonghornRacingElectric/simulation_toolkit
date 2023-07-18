from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters


class Car(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "Car"
        self._model_name = "Unnamed Car"

        # TODO these parameters are temporary for testing

        self.motor_max_torque = ConstantParameter()
        self.wheel_radius = ConstantParameter()
        self.mass = ConstantParameter()

        self._lock()
