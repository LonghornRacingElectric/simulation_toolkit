import numpy as np


class RealisticSensor:
    def __init__(self, precision: float, interval: float, delay: float, noise: float):
        """

        :param precision: precision of sensor output
        :param interval: sensor data collection interval (seconds)
        :param delay: sensor data transmission delay (seconds)
        :param noise: Gaussian noise standard deviation
        """

        self.precision = precision
        self.interval = interval
        self.delay = delay

        self.noise_gen = np.random.default_rng()
        self.noise_std = noise

        self.times = []
        self.values = []

    def eval(self, time: float, value) -> any:
        if (not self.times) or (time >= self.times[-1] + self.interval):
            self.times.append(time)
            self.values.append(round(value / self.precision) * self.precision)

        senseTime = time - self.delay
        while self.times[0] < senseTime:
            self.times.pop(0)
            self.values.pop(0)

        noise = self.noise_gen.normal(0, self.noise_std)

        return self.values[0] + noise
