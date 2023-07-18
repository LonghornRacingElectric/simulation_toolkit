from sim.system_models.vectors.vector import Vector


class SensorDataVector(Vector):
    def __init__(self):
        super().__init__()

        self.wheelspeed_fl = 0
        self.wheelspeed_fr = 0
        self.wheelspeed_bl = 0
        self.wheelspeed_fr = 0

        # TODO IMU, GPS, gyro, etc
