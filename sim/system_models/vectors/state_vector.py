from sim.system_models.vectors.vector import Vector


class StateVector(Vector):
    def __init__(self):
        super().__init__()

        self.hv_battery_charge = 0
        self.lv_battery_charge = 0

        self.motor_rpm = 0

        self.hv_battery_temperature = 0
        self.inverter_temperature = 0
        self.motor_temperature = 0
        self.coolant_temperature = 0
