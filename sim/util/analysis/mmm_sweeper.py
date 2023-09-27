import matplotlib.pyplot as plt

from sim.model_parameters.cars.car import Car
from sim.model_parameters.parameters import ConstantParameter
from sim.simulations.mmm_solver import MmmSolver


class MmmSweeper:
    def __init__(self, car: Car, mesh: int, velocity: float, aero: bool):
        self.car = car
        self.mmm_solver = MmmSolver(car=car, mesh=mesh, velocity=velocity, aero=aero)
        self.dimensions = []
        self.results = []

    def add_dimension(self, param_name: str, values):
        self.dimensions.append((param_name, values))

    def solve_all(self):
        def solve_all_recursive(dim):
            if dim < len(self.dimensions):
                res = []
                dim_name, dim_values = self.dimensions[dim]
                for i in range(len(dim_values)):
                    self.car[dim_name] = ConstantParameter(dim_values[i])
                    res.append(solve_all_recursive(dim + 1))
                return res
            else:
                self.mmm_solver.solve()
                return self.mmm_solver.key_points

        self.results = solve_all_recursive(0)

    def _plot_key_point(self, dimension: str, kpi: int):
        if len(self.dimensions) > 1:
            raise Exception("plotting multidimensional sweeps isn't implemented yet because then you have to\
                choose a value for the other dimensions and i don't feel like setting that up rn sorry - matt")

        x = None
        for dim in self.dimensions:
            if dim[0] == dimension:
                x = dim[1]
                break
        if x is None:
            raise Exception("dimension not found")

        y = [kp[kpi][1] for kp in self.results]

        plt.title(f'{self.results[0][kpi][0]} vs {dimension}')
        plt.xlabel(dimension)
        plt.ylabel(f'{self.results[0][kpi][0]} ({self.results[0][kpi][2]})')
        # plt.axhline(c="gray", linewidth=0.5)
        plt.plot(x, y, c="red", linewidth=0.8)

        plt.show()

    def plot_linear_control(self, dimension: str):
        self._plot_key_point(dimension, 0)

    def plot_linear_stability(self, dimension: str):
        self._plot_key_point(dimension, 1)

    def plot_max_lat_accel(self, dimension: str):
        self._plot_key_point(dimension, 2)

    def plot_limit_yaw_stability(self, dimension: str):
        self._plot_key_point(dimension, 3)

    def plot_max_yaw_accel(self, dimension: str):
        self._plot_key_point(dimension, 4)

    def plot_trim_lat_accel(self, dimension: str):
        self._plot_key_point(dimension, 5)

    def plot_all(self, dimension: str):
        for i in range(6):
            self._plot_key_point(dimension, i)
