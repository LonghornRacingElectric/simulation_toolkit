import numpy as np

from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.drivers.driver import Driver
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class RylanHanks(Driver):
    def __init__(self):
        super().__init__()
        self._model_name = "Rylan Hanks"

        self.acceleration_confidence = ConstantParameter(0.5)
        self.braking_confidence = ConstantParameter(1.0)
        self.cornering_confidence = ConstantParameter(1.0)

    def driver_program(self, time: float, state: StateVector, state_dot: StateDotVector) -> DriverControlsVector:
        out = DriverControlsVector()

        if time < 0.5:
            out.accel_pedal_pct = 0
            out.brake_pedal_pct = 0.5
            out.drive_switch = False
        elif time < 0.8:
            out.accel_pedal_pct = 0
            out.brake_pedal_pct = 0.5
            out.drive_switch = True
        elif time < 1.0:
            out.accel_pedal_pct = 0
            out.brake_pedal_pct = 0
            out.drive_switch = True
        else:
            out.accel_pedal_pct = 1.0
            out.brake_pedal_pct = 0
            out.drive_switch = True

        # keep straight
        out.steering_angle = 5 * state.body_slip

        # if we spin out just stop the sim
        if abs(state.yaw_rate) > 2*np.pi * 3:
            out.e_stop = True

        # if we finish acceleration event stop sim
        if state.displacement[0] > 75:
            out.e_stop = True

        return out
