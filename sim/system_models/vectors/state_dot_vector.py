from sim.system_models.vectors.vector import Vector


class StateDotVector(Vector):
    def __init__(self):
        super().__init__()

        self.hv_battery_current = 0
        self.lv_battery_current = 0

        self.motor_torque = 0

        self.hv_battery_net_heat = 0
        self.inverter_net_heat = 0
        self.motor_net_heat = 0
        self.coolant_net_heat = 0

        self.applied_torque_fl = 0
        self.applied_torque_fr = 0
        self.applied_torque_bl = 0
        self.applied_torque_br = 0
