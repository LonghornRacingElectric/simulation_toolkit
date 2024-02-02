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

        self.fault_apps = False
        self.fault_bse = False
        self.fault_stompp = False
        self.fault_steering = False
