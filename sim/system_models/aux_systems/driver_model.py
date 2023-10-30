"""

Transient driver model.

Remember that the driver needs to know the racing line to try to match it in a transient sim.
For a simple scenario (like a single turn) the driver can be trying to follow a simple spline instead.
Alternatively, we can have the driver be following "instructions" instead of a path,
like "accelerate for 1 second, then turn steering wheel 10 degrees to the right for 1 second".

Driver also needs to know the performance envelope so they can try to operate near its boundaries.
This implies that the quasi-steady state sim needs to be done before trying to model laps in a transient sim.

"""
import numpy as np

from sim.model_parameters.drivers.driver import Driver
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.util.math.conversions import deg_to_rad


class DriverModel:
    def __init__(self, driver_parameters: Driver):
        self.parameters = driver_parameters

    def eval(self, time: float, state: StateVector, state_dot: StateDotVector) -> DriverControlsVector:
        out = DriverControlsVector()

        if time < 0.5:
            out.accel_pedal_pct = 0
            out.brake_pedal_pct = 0.5
            out.drive_switch = False
        elif time < 1.0:
            out.accel_pedal_pct = 0
            out.brake_pedal_pct = 0.5
            out.drive_switch = True
        else:
            out.accel_pedal_pct = 0.3
            out.brake_pedal_pct = 0
            out.drive_switch = True
            out.steering_angle = 0

            if 3 < time < 4:
                out.steering_angle = deg_to_rad(-30) * (time - 3)
            elif 4 < time < 5:
                out.steering_angle = deg_to_rad(-30) * (5 - time)
            elif 5 < time < 6:
                out.steering_angle = deg_to_rad(15) * (time - 5)
            elif 6 < time < 7:
                out.steering_angle = deg_to_rad(15) * (7 - time)

        # if we spin out just stop the sim
        if abs(state.yaw_rate) > 2*np.pi * 3:
            out.e_stop = True

        return out
