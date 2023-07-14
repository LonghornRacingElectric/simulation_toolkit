from sim.model_parameters.parameters.toggle_parameter import ToggleParameter
from sim.model_parameters.model_parameters import ModelParameters


class VehicleControlUnit(ModelParameters):
    def __init__(self):
        super().__init__()
        self.model_type = "VCU"
        self.model_name = "Unnamed VCU"
        self._parameters = {
            "traction_control_enabled": type(ToggleParameter()),
            "anti_lock_brakes_enabled": type(ToggleParameter())
        }
