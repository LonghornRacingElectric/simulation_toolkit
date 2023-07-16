from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.model_parameters import ModelParameters


class Car(ModelParameters):
    def __init__(self):
        super().__init__()
        self.model_type = "Car"
        self.model_name = "Unnamed Car"
        self._parameters = {
            "max_torque": type(ConstantParameter()),
            "wheel_radius": type(ConstantParameter()),
            "mass": type(ConstantParameter())
        }

        # TODO these parameters are temporary for testing
