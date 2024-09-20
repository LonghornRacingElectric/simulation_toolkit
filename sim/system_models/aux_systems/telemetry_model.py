"""

Telemetry model takes all real data and turns it into sensor data. Sensors are never ideal,
and those imperfections are modeled here. This file also handles any control input transducers like
the APPS and BSE sensors.

An example would be a wheelspeed sensor only detecting discrete increments in wheel angle.

There's also a bit of a delay for the data to get transmitted over CAN to the VCU.

"""
import numpy as np

from sim.model_parameters.telemetry.telemetry import Telemetry
from sim.system_models.sensor_models.realistic_sensor import RealisticSensor
from sim.system_models.sensor_models.wheel_speed_sensor import WheelSpeedSensor
from sim.system_models.vectors.driver_controls_vector import DriverControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.sensor_data_vector import SensorDataVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class TelemetryModel:
    def __init__(self, telemetry_parameters: Telemetry):
        self.parameters = telemetry_parameters

        self.apps1_sensor = RealisticSensor(0.001, 0.003, 0.001, 0.002)
        self.apps2_sensor = RealisticSensor(0.001, 0.003, 0.001, 0.002)
        self.bse1_sensor = RealisticSensor(0.001, 0.003, 0.001, 0.002)
        self.bse2_sensor = RealisticSensor(0.001, 0.003, 0.001, 0.002)

        self.wheel_fl_sensor = WheelSpeedSensor(0.0025, 0.003, 0.005, 0.1)
        self.wheel_fr_sensor = WheelSpeedSensor(0.0025, 0.003, 0.005, 0.1)
        self.wheel_bl_sensor = WheelSpeedSensor(0.0025, 0.003, 0.005, 0.1)
        self.wheel_br_sensor = WheelSpeedSensor(0.0025, 0.003, 0.005, 0.1)

    def eval(self, time: float, state: StateVector, state_dot: StateDotVector, observables: ObservablesVector,
             driver_controls: DriverControlsVector) -> SensorDataVector:

        sensor_data = SensorDataVector()

        sensor_data.apps1 = self.apps1_sensor.eval(time, 2.24 + driver_controls.accel_pedal_pct * 0.70)
        sensor_data.apps2 = self.apps2_sensor.eval(time, 1.08 + driver_controls.accel_pedal_pct * 0.38)
        sensor_data.bse1 = self.bse1_sensor.eval(time, 0.50 + driver_controls.brake_pedal_pct * 2.0)
        sensor_data.bse2 = self.bse2_sensor.eval(time, 0.50 + driver_controls.brake_pedal_pct * 2.0)

        sensor_data.drive_switch = driver_controls.drive_switch

        sensor_data.wheel_magnetic_field_fl = self.wheel_fl_sensor.eval(time, state.wheel_angular_displacements[0])
        sensor_data.wheel_magnetic_field_fr = self.wheel_fr_sensor.eval(time, state.wheel_angular_displacements[1])
        sensor_data.wheel_magnetic_field_bl = self.wheel_bl_sensor.eval(time, state.wheel_angular_displacements[2])
        sensor_data.wheel_magnetic_field_br = self.wheel_br_sensor.eval(time, state.wheel_angular_displacements[3])

        sensor_data.motor_rpm = state.motor_rpm

        sensor_data.inverter_ready = True

        sensor_data.hv_battery_voltage = observables.hv_battery_terminal_voltage
        sensor_data.hv_battery_current = state_dot.hv_battery_current

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
