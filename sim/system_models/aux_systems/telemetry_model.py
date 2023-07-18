"""

TODO true vehicle state (x, x_dot) in, realistic sensor data out

An example would be a wheelspeed sensor only detecting discrete increments in wheel angle.

There's also a bit of a delay for the data to get transmitted over CAN to the VCU.

"""
from sim.model_parameters.telemetry.telemetry import Telemetry
from sim.system_models.vectors.sensor_data_vector import SensorDataVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class TelemetryModel:
    def __init__(self, telemetry_parameters: Telemetry):
        self.parameters = telemetry_parameters

    def eval(self, state: StateVector, state_dot: StateDotVector) -> SensorDataVector:
        sensor_data = SensorDataVector()

        # TODO implement

        return sensor_data
