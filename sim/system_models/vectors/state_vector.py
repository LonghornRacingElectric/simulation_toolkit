import numpy as np

from sim.system_models.vectors.vector import Vector


class StateVector(Vector):
    def __init__(self):
        super().__init__()

        self.hv_battery_charge = 0
        self.lv_battery_charge = 0

        self.hv_battery_temperature = 0
        self.inverter_temperature = 0
        self.motor_temperature = 0
        self.coolant_temperature = 0
        
        self.heave = 0
        self.pitch = 0
        self.roll = 0

        # note that velocity and displacement are global, not IMF/NTB
        self.velocity = np.array([0, 0, 0], dtype=float)
        self.displacement = np.array([0, 0, 0], dtype=float)
        self.yaw_rate = 0
        self.yaw = 0

        self.motor_rpm = 0

        self.wheel_slip_ratios = np.array([0, 0, 0, 0], dtype=float)
        self.wheel_angular_velocities = np.array([0, 0, 0, 0], dtype=float)
        self.wheel_angular_displacements = np.array([0, 0, 0, 0], dtype=float)

        self.body_slip = 0
        self.speed = 0
