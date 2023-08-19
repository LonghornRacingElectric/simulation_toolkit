"""

This is like CurveParameter but when you have two inputs instead of just one. Hence, the function is a 3D surface.

"""

from typing import Callable
import numpy as np
import csv

from sim.model_parameters.parameters.parameter import Parameter


# TODO implement

class SurfaceParameter(Parameter):
    def __init__(self, from_csv: str = None, from_function: Callable[[float, float], float] = None,
                 from_points: list[tuple[float, float, float]] = None,
                 x_min: float = None, x_max: float = None, x_samples: int = None,
                 y_min: float = None, y_max: float = None, y_samples: int = None):
        self.x_arr = None
        self.y_arr = None
        self.z_arr = None

        if from_csv:
            self._import_from_csv(from_csv)
        elif from_function:
            self._import_from_function(from_function, x_min, x_max, x_samples, y_min, y_max, y_samples)
        elif from_points:
            self._import_from_points(from_points)
        else:
            pass

    def _import_from_csv(self, csv_path: str):
        raise NotImplementedError()

        # x_list, y_list, z_list = [], [], []
        # with open(csv_path, newline='') as csvfile:
        #     reader = csv.reader(csvfile)
        #     first = True
        #     for row in reader:
        #         if first:
        #             first = False
        #             continue
        #         if len(row) != 3:
        #             raise Exception(f"Expected 3 items per row in CSV file {csv_path} but found {len(row)}.")
        #         x_list.append(float(row[0]))
        #         y_list.append(float(row[0]))
        #         z_list.append(float(row[0]))
        # self.x_arr = np.array(x_list)
        # self.y_arr = np.array(y_list)
        # self.z_arr = np.array(z_list)

    def _import_from_function(self, function: Callable[[float, float], float],
                              x_min: float, x_max: float, x_samples: int,
                              y_min: float, y_max: float, y_samples: int):
        raise NotImplementedError()

        # if (x_min is None) or (x_max is None) or (x_samples is None) \
        #         or (y_min is None) or (y_max is None) or (y_samples is None):
        #     raise Exception("If creating a SurfaceParameter from a function, provide x/y_min, x/y_max, and x/y_samples")
        # x_list, y_list, z_list = [], [], []
        # for i in range(x_samples + 1):
        #     x = x_min + ((x_max - x_min) * (i / x_samples))
        #     for j in range(y_samples + 1):
        #         y = y_min + ((y_max - y_min) * (j / y_samples))
        #         z = function(x, y)
        #         x_list.append(x)
        #         y_list.append(y)
        #         z_list.append(z)
        # self.x_arr = np.array(x_list)
        # self.y_arr = np.array(y_list)
        # self.z_arr = np.array(z_list)

    def _import_from_points(self, points: list[tuple[float, float, float]]):
        raise NotImplementedError()

        # self.x_arr = np.array([row[0] for row in points])
        # self.y_arr = np.array([row[1] for row in points])
        # self.z_arr = np.array([row[2] for row in points])

    def _sample(self, x: float, y: float):
        raise NotImplementedError()

    # returns a function
    def get(self):
        return self._sample

    def is_set(self) -> bool:
        return (self.x_arr is not None) and (self.y_arr is not None) and (self.z_arr is not None)
