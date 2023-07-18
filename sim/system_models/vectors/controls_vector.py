from sim.system_models.vectors.vector import Vector


class ControlsVector(Vector):
    def __init__(self):
        super().__init__()

        self.torque_request = 0
        self.brake_pedal_pressure = 0
        self.steering_angle = 0
