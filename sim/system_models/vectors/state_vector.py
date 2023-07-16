from sim.system_models.vectors.vector import Vector


class StateVector(Vector):
    def __init__(self):
        super().__init__()

        self.velocity = 0
