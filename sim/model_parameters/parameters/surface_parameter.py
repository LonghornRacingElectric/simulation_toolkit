"""

This is like CurveParameter but when you have two inputs instead of just one. Hence, the function is a 3D surface.

"""
from typing import Callable
import numpy as np
import csv

from sim.model_parameters.parameters.parameter import Parameter
from sim.util.math.conversions import get_csv_path


class SurfaceParameter(Parameter):
    def __init__(self, from_csv: str = None, from_function: Callable[[float, float], float] = None,
                 x_min: float = None, x_max: float = None, x_samples: int = None,
                 y_min: float = None, y_max: float = None, y_samples: int = None):
        self.x_arr = None
        self.y_arr = None
        self.data = None

        if from_csv:
            self._import_from_csv(from_csv)
        elif from_function:
            self._import_from_function(from_function, x_min, x_max, x_samples, y_min, y_max, y_samples)
        else:
            pass

    def _import_from_csv(self, csv_path: str):
        with open(get_csv_path(csv_path), newline='') as csvfile:
            reader = csv.reader(csvfile)
            x_list = []
            y_list = []
            self.data = []
            first = True
            for row in reader:
                if first:
                    first = False
                    x_list = list(map(float, row[1:]))
                    continue
                y_list.append(float(row[0]))
                self.data.append(list(map(float, row[1:])))
            self.x_arr = np.array(x_list)
            self.y_arr = np.array(y_list)

    def _import_from_function(self, function: Callable[[float, float], float],
                              x_min: float, x_max: float, x_samples: int,
                              y_min: float, y_max: float, y_samples: int):
        if (x_min is None) or (x_max is None) or (x_samples is None) \
                or (y_min is None) or (y_max is None) or (y_samples is None):
            raise Exception("If creating a SurfaceParameter from a function, provide x/y_min, x/y_max, and x/y_samples")
        self.x_arr = np.linspace(x_min, x_max, num=x_samples)
        self.y_arr = np.linspace(y_min, y_max, num=y_samples)
        self.data = [[function(x, y) for x in self.x_arr] for y in self.y_arr]

    def _binary_search(self, arr, n) -> int:
        lo, hi = 0, len(arr)
        while lo + 1 < hi:
            mi = (lo + hi) // 2
            m = arr[mi]
            if n > m:
                lo = mi
            elif n < m:
                hi = mi
            else:
                return mi
        return lo

    def _sample(self, x: float, y: float):
        xi = self._binary_search(self.x_arr, x)
        yi = self._binary_search(self.y_arr, y)

        xj = xi + 1
        yj = yi + 1
        if xj == len(self.x_arr):
            xj = xi
        if yj == len(self.y_arr):
            yj = yi

        xp = 0 if (xi == xj) else ((x - self.x_arr[xi]) / (self.x_arr[xj] - self.x_arr[xi]))
        yp = 0 if (yi == yj) else ((y - self.y_arr[yi]) / (self.y_arr[yj] - self.y_arr[yi]))

        z1 = (self.data[yi][xi] * (1 - xp)) + (self.data[yi][xj] * xp)
        z2 = (self.data[yj][xi] * (1 - xp)) + (self.data[yj][xj] * xp)
        z = (z1 * (1 - yp)) + (z2 * yp)

        return z

    # returns a function
    def get(self):
        return self._sample

    def is_set(self) -> bool:
        return (self.x_arr is not None) and (self.y_arr is not None) and (self.data is not None)
