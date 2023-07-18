from sim.model_parameters.parameters.constant_parameter import ConstantParameter
from sim.model_parameters.drivers.driver import Driver


class BenHuff(Driver):
    def __init__(self):
        super().__init__()
        self._model_name = "Ben Huff"

        self.acceleration_confidence = ConstantParameter(1.0)
        self.braking_confidence = ConstantParameter(0.1)
        self.cornering_confidence = ConstantParameter(0.8)
        # data is from the following paper: https://lhre.slack.com/files/U02E2NBSTCH/F05A78N20P4/vid_20230530_224750.mp4
