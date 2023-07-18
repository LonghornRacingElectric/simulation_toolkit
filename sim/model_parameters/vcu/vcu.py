from sim.model_parameters.parameters.toggle_parameter import ToggleParameter
from sim.model_parameters.model_parameters import ModelParameters


class VehicleControlUnit(ModelParameters):
    def __init__(self):
        super().__init__()
        self._model_type = "VCU"
        self._model_name = "Unnamed VCU"

        self.traction_control_enabled = ToggleParameter()
        self.anti_lock_brakes_enabled = ToggleParameter()

        self._lock()
