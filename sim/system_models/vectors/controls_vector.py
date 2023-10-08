import numpy as np

from sim.system_models.vectors.vector import Vector


class ControlsVector(Vector):
    def __init__(self):
        super().__init__()

        self.torque_request = 0
        self.enable_drs = False

        self.brake_pct = 0
        self.steering_angle = 0
        self.cooling_percent = 0
