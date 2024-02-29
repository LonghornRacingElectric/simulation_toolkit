import os
from subprocess import Popen, PIPE, STDOUT

import numpy as np

from sim.system_models.vectors.sensor_data_vector import SensorDataVector
from sim.system_models.vectors.vcu_output_vector import VcuOutputVector

output_line_count = 33


class VcuCoreProcess:
    def __init__(self):
        executable_path = os.path.join(os.path.dirname(__file__), 'vcu_core.exe')
        self.process = Popen([executable_path], bufsize=0, stdin=PIPE, stdout=PIPE)
        self.input_lines = []
        self.output_lines = []

        self.x = 0

    def execute(self, sensor_data: SensorDataVector) -> VcuOutputVector:
        self.input_lines = []
        self.output_lines = []
        self.write_all(sensor_data)
        self.communicate()
        return self.read_all()

    def terminate_subprocess(self):
        self.process.terminate()

    def communicate(self):
        for line in self.input_lines:
            self.process.stdin.write(bytes(line, 'utf-8'))
            self.process.stdin.write(bytes('\n', 'utf-8'))
        self.process.stdin.flush()

        self.output_lines = [self.process.stdout.readline().decode('utf-8').strip() for _ in range(output_line_count)]

    def write_all(self, sensor_data: SensorDataVector):
        self.write_float(sensor_data.apps1)
        self.write_float(sensor_data.apps2)

        self.write_float(sensor_data.bse1)
        self.write_float(sensor_data.bse2)

        self.write_float(sensor_data.steering_wheel_pot_voltage)

        self.write_float(sensor_data.wheel_magnetic_field_fl)
        self.write_float(sensor_data.wheel_magnetic_field_fr)
        self.write_float(sensor_data.wheel_magnetic_field_bl)
        self.write_float(sensor_data.wheel_magnetic_field_br)

        self.write_float(sensor_data.motor_temp)
        self.write_float(sensor_data.inverter_temp)
        self.write_float(sensor_data.battery_temp)

        self.write_float(sensor_data.hv_battery_soc)
        self.write_bool(sensor_data.inverter_ready)

        self.write_float(sensor_data.hv_battery_voltage)
        self.write_float(sensor_data.hv_battery_current)

        self.write_float(sensor_data.lv_battery_voltage)
        self.write_float(sensor_data.lv_battery_current)

        self.write_bool(sensor_data.drive_switch)

        self.write_float(sensor_data.gps_lat)
        self.write_float(sensor_data.gps_long)
        self.write_float(sensor_data.gps_speed)
        self.write_float(sensor_data.gps_heading)

        self.write_vector(sensor_data.imuAccel1)
        self.write_vector(sensor_data.imuAccel2)
        self.write_vector(sensor_data.imuAccel3)
        self.write_vector(sensor_data.imuGyro1)
        self.write_vector(sensor_data.imuGyro2)
        self.write_vector(sensor_data.imuGyro3)

    def read_all(self) -> VcuOutputVector:
        out = VcuOutputVector()

        out.enable_inverter = self.read_bool()
        out.torque_request = self.read_float()

        out.park_or_drive = self.read_bool()
        out.r2d_buzzer = self.read_bool()
        out.brake_light = self.read_bool()

        out.enable_drs = self.read_bool()

        out.pump_output = self.read_float()
        out.rad_fans_output = self.read_float()
        out.batt_fans_output = self.read_float()

        out.estimated_displacement = self.read_vector()
        out.estimated_velocity = self.read_vector()
        out.estimated_acceleration = self.read_vector()

        out.estimated_hv_soc = self.read_float()
        out.estimated_lv_soc = self.read_float()

        out.dash_speedometer = self.read_float()

        out.apps1 = self.read_float()
        out.apps2 = self.read_float()
        out.apps = self.read_float()
        out.bse1 = self.read_float()
        out.bse2 = self.read_float()
        out.bse = self.read_float()
        out.estimated_wheel_speed_fl = self.read_float()
        out.estimated_wheel_speed_fr = self.read_float()
        out.estimated_wheel_speed_bl = self.read_float()
        out.estimated_wheel_speed_br = self.read_float()
        out.estimated_steering_wheel_angle = self.read_float()

        out.flags = self.read_int()

        return out

    def write_float(self, x: float):
        self.input_lines.append(str(x))

    def write_bool(self, b: bool):
        self.input_lines.append(str(int(b)))

    def write_vector(self, v):
        for x in v:
            self.write_float(x)

    def read_line(self) -> str:
        s = self.output_lines.pop(0)
        return s

    def read_float(self) -> float:
        x = float(self.read_line())
        return x

    def read_bool(self) -> bool:
        x = float(self.read_line())
        return x != 0

    def read_int(self) -> int:
        x = int(self.read_line())
        return x

    def read_vector(self) -> np.array:
        x = self.read_float()
        y = self.read_float()
        z = self.read_float()
        return np.array([x, y, z])
