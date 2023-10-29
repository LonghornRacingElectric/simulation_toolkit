import matplotlib.pyplot as plt
import numpy as np

from sim.model_parameters.cars.car import Car
from sim.model_parameters.parameters import ConstantParameter
from sim.simulations.mmm_solver import MmmSolver


class MmmSweeper:
    def __init__(self, car: Car, mesh: int, velocity: float):
        self.car = car
        self.mmm_solver = MmmSolver(car=car, mesh=mesh, velocity=velocity)
        self.dimensions = []
        self.results = []

    def add_dimension(self, param_name: str, values, couple_to: str = ""):
        """
        :param param_name: Name of the vehicle parameter you want to sweep.
        :param values: Array of values you want to sweep over.
        :param couple_to: (Optional) Parameter you want to couple this to. Values array length must match.
        """
        if couple_to:
            for dim_name_list, dim_values_list in self.dimensions:
                if couple_to in dim_name_list:
                    dim_name_list.append(param_name)
                    dim_values_list.append(values)
                    assert len(dim_values_list[0]) == len(values)
                    break
        else:
            self.dimensions.append(([param_name], [values]))

    def update_velocity(self, velocity: float):
        self.mmm_solver.velocity = velocity

    def solve_all(self):
        def solve_all_recursive(dim, input_values):
            if dim < len(self.dimensions):
                res = []
                dim_name_list, dim_values_list = self.dimensions[dim]
                for i in range(len(dim_values_list[0])):
                    new_input_values = input_values.copy()
                    for dim_name, dim_values in zip(dim_name_list, dim_values_list):
                        self.car[dim_name] = ConstantParameter(dim_values[i])
                        new_input_values[dim_name] = dim_values[i]
                    res.append(solve_all_recursive(dim + 1, new_input_values))
                return res
            else:
                self.mmm_solver.solve()

                print()
                print("=============== single MMM finished ===============")
                for k, v in input_values.items():
                    print(f'{k.ljust(25)} | {self._format(v)}')
                print('---------------------------------------------------')
                self.mmm_solver.print_key_points()
                print()

                return self.mmm_solver.key_points

        self.results = solve_all_recursive(0, {})

    def _plot_key_point(self, dimension: str, kpi: int):
        x = None
        for dim in self.dimensions:
            if dimension in dim[0]:
                x = dim[1][dim[0].index(dimension)]
                if not isinstance(x[0], np.floating):
                    N = len(self.results) - 1
                    if not N:
                        N = 1
                    x = np.array([i / N for i in range(N + 1)])
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

    def plot_key_points(self, dimension: str):
        for i in range(6):
            self._plot_key_point(dimension, i)

    def plot_convex_hull(self, dimension: str):
        d = None
        for dim in self.dimensions:
            if dimension in dim[0]:
                i = dim[0].index(dimension)
                d = (dim[0][i], dim[1][i])
                break
        if d is None:
            raise Exception("dimension not found")

        fig = plt.figure()
        subplot = fig.add_subplot()

        N = len(self.results) - 1
        if not N:
            N = 1

        for i, res in enumerate(self.results):
            x, y = res[6]
            x = list(x) + [x[0]]  # wrap around
            y = list(y) + [y[0]]
            z = d[1][i]
            c = i / N
            subplot.plot(x, y, label=f'{d[0]} = {self._format(z)}', color=(c, 0, 1 - c))

        subplot.set_xlabel('Lateral Acceleration (m/s^2)')
        subplot.set_ylabel('Yaw Acceleration (rad/s^2)')
        plt.axhline(c="gray", linewidth=0.5)
        plt.axvline(c="gray", linewidth=0.5)
        subplot.legend()

        plt.show()

    def _format(self, value):
        if isinstance(value, np.floating):
            return str(round(value, 3))
        else:
            return "[" + ", ".join([self._format(v) for v in value]) + "]"
