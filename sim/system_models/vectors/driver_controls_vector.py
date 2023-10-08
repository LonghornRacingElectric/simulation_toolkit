from sim.system_models.vectors.vector import Vector


class DriverControlsVector(Vector):
    def __init__(self):
        super().__init__()

        self.e_stop = False
        self.drive_switch = False
        self.accel_pedal_pct = 0
        self.brake_pedal_pct = 0
        self.steering_angle = 0
