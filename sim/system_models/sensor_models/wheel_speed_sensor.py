import numpy as np

from sim.system_models.sensor_models.realistic_sensor import RealisticSensor


class WheelSpeedSensor(RealisticSensor):
    def __init__(self, precision: float, interval: float, delay: float, noise: float):
        """

        :param precision: precision of sensor output
        :param interval: sensor data collection interval (seconds)
        :param delay: sensor data transmission delay (seconds)
        :param noise: Gaussian noise standard deviation
        """

        super().__init__(precision, interval, delay, noise)

    def eval(self, time: float, angle) -> any:
        value = np.sin(angle*3) * 1.0
        return super().eval(time, value)
