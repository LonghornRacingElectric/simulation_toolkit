import numpy as np

from sim.system_models.vectors.vector import Vector


class VcuOutputVector(Vector):
    def __init__(self):
        super().__init__()

        self.enable_inverter = False
        self.torque_request = 0

        self.park_or_drive = False
        self.r2d_buzzer = False
        self.brake_light = False

        self.enable_drs = False

        self.pump_output = 0
        self.rad_fans_output = 0
        self.batt_fans_output = 0

        self.estimated_displacement = np.array([0, 0, 0])
        self.estimated_velocity = np.array([0, 0, 0])
        self.estimated_acceleration = np.array([0, 0, 0])

        self.estimated_hv_soc = 0
        self.estimated_lv_soc = 0

        self.dash_speedometer = 0

        self.apps1 = 0
        self.apps2 = 0
        self.apps = 0
        self.bse1 = 0
        self.bse2 = 0
        self.bse = 0
        self.estimated_wheel_speed_fl = 0
        self.estimated_wheel_speed_fr = 0
        self.estimated_wheel_speed_bl = 0
        self.estimated_wheel_speed_br = 0
        self.estimated_steering_wheel_angle = 0

        self.flags = 0
