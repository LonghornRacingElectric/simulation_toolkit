"""

Transient driver model.

Remember that the driver needs to know the racing line to try to match it in a transient sim.
For a simple scenario (like a single turn) the driver can be trying to follow a simple spline instead.
Alternatively, we can have the driver be following "instructions" instead of a path,
like "accelerate for 1 second, then turn steering wheel 10 degrees to the right for 1 second".

Driver also needs to know the performance envelope so they can try to operate near its boundaries.
This implies that the quasi-steady state sim needs to be done before trying to model laps in a transient sim.

"""

from sim.model_parameters.drivers.driver import Driver
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class DriverModel:
    def __init__(self, driver_parameters: Driver):
        self.parameters = driver_parameters

    def eval(self, time: float, state: StateVector, state_dot: StateDotVector) -> DriverControlsVector:
        return self.parameters.driver_program(time, state, state_dot)
