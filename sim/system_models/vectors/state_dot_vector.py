import numpy as np

from sim.system_models.vectors.vector import Vector


class StateDotVector(Vector):
    def __init__(self):
        super().__init__()

        self.hv_battery_current = 0
        self.lv_battery_current = 0

        self.hv_battery_net_heat = 0
        self.inverter_net_heat = 0
        self.motor_net_heat = 0
        self.coolant_net_heat = 0

        self.powertrain_torques = np.array([0, 0, 0, 0])
        self.tire_torques = np.array([0, 0, 0, 0])

        self.sus_forces = np.array([0, 0, 0])
        self.sus_moments = np.array([0, 0, 0])

        self.aero_forces = np.array([0, 0, 0])
        self.aero_moments = np.array([0, 0, 0])
