"""

Some vehicle parameters are too complex to just be defined by constant values. This class allows you to use a
function as a parameter instead.

For an example, think about how a nonlinear spring's stiffness may be a function of its displacement,
or how a battery voltage may be a function of its state of charge. These relationships can be imported as data from a
CSV file, from a list of points, or from a Python function.

"""

from typing import Callable
import numpy as np
import csv
import os

from sim.model_parameters.parameters.parameter import Parameter
from sim.util.math.conversions import get_csv_path


class CurveParameter(Parameter):
    def __init__(self, from_csv: str = None, from_function: Callable[[float], float] = None,
                 from_points: list[tuple[float, float]] = None,
                 x_min: float = None, x_max: float = None, x_samples: int = None):
        self.x_arr = None
        self.y_arr = None

        if from_csv:
            self._import_from_csv(from_csv)
        elif from_function:
            self._import_from_function(from_function, x_min, x_max, x_samples)
        elif from_points:
            self._import_from_points(from_points)
        else:
            pass

    def _import_from_csv(self, csv_path: str):
        x_list, y_list = [], []

        with open(get_csv_path(csv_path), newline='') as csvfile:
            reader = csv.reader(csvfile)
            first = True
            for row in reader:
                if first:
                    first = False
                    continue
                if len(row) != 2:
                    raise Exception(f"Expected 2 items per row in CSV file {csv_path} but found {len(row)}.")
                x_list.append(float(row[0]))
                y_list.append(float(row[0]))
        self.x_arr = np.array(x_list)
        self.y_arr = np.array(y_list)

        if not np.all(np.diff(self.x_arr) >= 0):
            raise Exception(f"The x values are out of order in CSV file {csv_path}.")

    def _import_from_function(self, function: Callable[[float], float], x_min: float, x_max: float, x_samples: int):
        if (x_min is None) or (x_max is None) or (x_samples is None):
            raise Exception("If creating a CurveParameter from a function, provide x_min, x_max, and x_samples")
        if x_min > x_max:
            raise Exception("Why is x_min greater than x_max??")
        x_list, y_list = [], []
        for i in range(x_samples):
            x = x_min + ((x_max - x_min) * (i / (x_samples - 1)))
            y = function(x)
            x_list.append(x)
            y_list.append(y)
        self.x_arr = np.array(x_list)
        self.y_arr = np.array(y_list)

    def _import_from_points(self, points: list[tuple[float, float]]):
        self.x_arr = np.array([row[0] for row in points])
        self.y_arr = np.array([row[1] for row in points])

        if not np.all(np.diff(self.x_arr) >= 0):
            raise Exception(f"The x values are out of order in the points provided to this CurveParameter.")

    def _sample(self, x: float):
        return np.interp(x, self.x_arr, self.y_arr)

    # returns a function
    def get(self):
        return self._sample

    def is_set(self) -> bool:
        return (self.x_arr is not None) and (self.y_arr is not None)
