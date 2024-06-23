"""

Telemetry model takes all real data and turns it into sensor data. Sensors are never ideal,
and those imperfections are modeled here. This file also handles any control input transducers like
the APPS and BSE sensors.

An example would be a wheelspeed sensor only detecting discrete increments in wheel angle.

There's also a bit of a delay for the data to get transmitted over CAN to the VCU.

"""
import numpy as np

from sim.model_parameters.telemetry.telemetry import Telemetry
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.sensor_data_vector import SensorDataVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class TelemetryModel:
    def __init__(self, telemetry_parameters: Telemetry):
        self.parameters = telemetry_parameters

    def eval(self, state: StateVector, state_dot: StateDotVector, observables: ObservablesVector,
             driver_controls: DriverControlsVector) -> SensorDataVector:

        sensor_data = SensorDataVector()

        # TODO implement
        sensor_data.apps1 = 1.0 + driver_controls.accel_pedal_pct * 3.0
        sensor_data.apps2 = 0.5 + driver_controls.accel_pedal_pct * 1.5
        sensor_data.bse1 = 0.5 + driver_controls.brake_pedal_pct * 4.0
        sensor_data.bse2 = 0.5 + driver_controls.brake_pedal_pct * 4.0

        sensor_data.drive_switch = driver_controls.drive_switch

        # TODO wheelspeeds won't respond instantly
        sensor_data.wheel_displacement_fl = state.wheel_angular_displacements[0]
        sensor_data.wheel_displacement_fr = state.wheel_angular_displacements[1]
        sensor_data.wheel_displacement_bl = state.wheel_angular_displacements[2]
        sensor_data.wheel_displacement_br = state.wheel_angular_displacements[3]

        sensor_data.motor_rpm = state.motor_rpm

        sensor_data.inverter_ready = True

        sensor_data.battery_voltage = observables.hv_battery_terminal_voltage
        sensor_data.battery_current = state_dot.hv_battery_current

        # TODO linearize position into GPS coordinates (with noise)
        sensor_data.gps_lat = 0
        sensor_data.gps_long = 0
        sensor_data.gps_speed = 0  # ignore for now
        sensor_data.gps_heading = 0  # ignore for now

        # TODO simulate 3 accelerometers based on positions in car (with noise)
        sensor_data.imuAccel1 = np.array([0, 0, 0])
        sensor_data.imuAccel2 = np.array([0, 0, 0])
        sensor_data.imuAccel3 = np.array([0, 0, 0])

        return sensor_data
