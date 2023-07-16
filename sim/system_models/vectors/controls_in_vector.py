from sim.system_models.vectors.vector import Vector


class ControlsInVector(Vector):
    def __init__(self):
        super().__init__()

        self.torque_request = 0
