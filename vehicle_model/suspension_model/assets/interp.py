from scipy.interpolate import interpn
from typing import Sequence
import numpy as np


class interp3d:
    def __init__(self, x: Sequence[Sequence[float]], y: Sequence[Sequence[float]], \
                 z: Sequence[Sequence[float]], w: Sequence[Sequence[float]], v: Sequence[Sequence[float]]) -> None:
        
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.v = v

    def __call__(self, x: float, y: float, z: float, w: float) -> float:
        points = (self.x, self.y, self.z, self.w)
        values = np.array(self.v).reshape((4, 4, 4, 4))
        return interpn(points=points, values=values, xi=(x, y, z, w))