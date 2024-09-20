import numpy as np

from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.drivers.driver import Driver
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.util.math.conversions import deg_to_rad


class BenHuff(Driver):
    def __init__(self):
        super().__init__()
        self._model_name = "Ben Huff"

        self.acceleration_confidence = ConstantParameter(1.0)
        self.braking_confidence = ConstantParameter(0.1)
        self.cornering_confidence = ConstantParameter(0.8)
        # data is from the following paper: https://lhre.slack.com/files/U02E2NBSTCH/F05A78N20P4/vid_20230530_224750.mp4

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
        elif time < 2:
            out.accel_pedal_pct = 0.5
            out.brake_pedal_pct = 0
            out.drive_switch = True
        else:
            out.accel_pedal_pct = 0.4
            out.brake_pedal_pct = 0
            out.drive_switch = True

            if time < 6:
                target_yaw = deg_to_rad(-90)
            else:
                target_yaw = 0

            target_yaw_rate = max(min(10*(target_yaw - state.yaw), deg_to_rad(50)), deg_to_rad(-50))
            out.steering_angle = max(min(8*(target_yaw_rate - state.yaw_rate), deg_to_rad(55)), deg_to_rad(-55))

        # if we spin out just stop the sim
        if abs(state.yaw_rate) > 2*np.pi * 3:
            out.e_stop = True

        return out
