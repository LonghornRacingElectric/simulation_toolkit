import numpy as np

from sim.system_models.vectors.vector import Vector


class SensorDataVector(Vector):
    def __init__(self):
        super().__init__()

        self.apps1 = 0
        self.apps2 = 0

        self.bse1 = 0
        self.bse2 = 0

        self.steering_wheel_pot_voltage = 0

        self.motor_rpm = 0

        self.wheel_displacement_fl = 0
        self.wheel_displacement_fr = 0
        self.wheel_displacement_bl = 0
        self.wheel_displacement_br = 0

        self.motor_temp = 0
        self.inverter_temp = 0
        self.battery_temp = 0

        self.battery_soc = 0
        self.inverter_ready = 0

        self.drive_switch = False

        self.battery_voltage = 0
        self.battery_current = 0

        self.gps_lat = 0
        self.gps_long = 0
        self.gps_speed = 0
        self.gps_heading = 0

        self.imuAccel1 = np.array([0, 0, 0])
        self.imuAccel2 = np.array([0, 0, 0])
        self.imuAccel3 = np.array([0, 0, 0])
        self.imuGyro1 = np.array([0, 0, 0])
        self.imuGyro2 = np.array([0, 0, 0])
        self.imuGyro3 = np.array([0, 0, 0])
