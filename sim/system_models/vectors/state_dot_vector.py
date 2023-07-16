from sim.system_models.vectors.vector import Vector


class StateDotVector(Vector):
    def __init__(self):
        super().__init__()

        self.acceleration = 0
