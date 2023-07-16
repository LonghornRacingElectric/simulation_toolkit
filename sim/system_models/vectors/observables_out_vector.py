from sim.system_models.vectors.vector import Vector


class ObservablesOutVector(Vector):
    def __init__(self):
        super().__init__()

        self.torque = 0
