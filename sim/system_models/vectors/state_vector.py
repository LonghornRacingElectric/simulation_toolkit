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
        
        self.heave = 0
        self.pitch = 0
        self.roll = 0
        self.lateral_accel = 0
        self.long_accel = 0
        self.yaw_accel = 0
        
        self.body_slip = 0
        self.steered_angle = 0
        self.velocity = 0
