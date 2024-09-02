from scipy.interpolate import interpn
from typing import Sequence
import numpy as np


class interp4d:
    def __init__(self, x: Sequence[Sequence[float]], y: Sequence[Sequence[float]], \
                 z: Sequence[Sequence[float]], w: Sequence[Sequence[float]], v: Sequence[Sequence[float]]) -> None:
        
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.v = v

    def __call__(self, x: float, y: float, z: float, w: float) -> float:
        points = (self.x, self.y, self.z, self.w)
        values = np.array(self.v).reshape((len(self.x), len(self.y), len(self.z), len(self.w)))

        if x > max(self.x):
            x = max(self.x)
        elif x < min(self.x):
            x = min(self.x)

        if y > max(self.y):
            y = max(self.y)
        elif y < min(self.y):
            y = min(self.y)

        if z > max(self.z):
            z = max(self.z)
        elif z < min(self.z):
            z = min(self.z)

        if w > max(self.w):
            w = max(self.w)
        elif w < min(self.w):
            w = min(self.w)

        return interpn(points=points, values=values, xi=(x, y, z, w))

class interp2d:
    def __init__(self, x: Sequence[Sequence[float]], y: Sequence[Sequence[float]], z: Sequence[Sequence[float]]) -> None:
        
        self.x = x
        self.y = y
        self.z = z

    def __call__(self, x: float, y: float) -> float:
        points = (self.x, self.y)
        values = np.array(self.z).reshape((len(self.x), len(self.y)))

        if x > max(self.x):
            x = max(self.x)
        elif x < min(self.x):
            x = min(self.x)

        if y > max(self.y):
            y = max(self.y)
        elif y < min(self.y):
            y = min(self.y)

        return interpn(points=points, values=values, xi=(x, y))