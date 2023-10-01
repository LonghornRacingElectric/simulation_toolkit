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
        def solve_all_recursive(dim, input_values):
            if dim < len(self.dimensions):
                res = []
                dim_name, dim_values = self.dimensions[dim]
                for i in range(len(dim_values)):
                    self.car[dim_name] = ConstantParameter(dim_values[i])
                    new_input_values = input_values.copy()
                    new_input_values[dim_name] = dim_values[i]
                    res.append(solve_all_recursive(dim + 1, new_input_values))
                return res
            else:
                self.mmm_solver.solve()

                print()
                print("=============== single MMM finished ===============")
                for k, v in input_values.items():
                    print(f'{k.ljust(25)} | {str(round(v, 3))}')
                print('---------------------------------------------------')
                self.mmm_solver.print_key_points()
                print()

                return self.mmm_solver.key_points

        self.results = solve_all_recursive(0, {})

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

    def plot_key_points(self, dimension: str):
        for i in range(6):
            self._plot_key_point(dimension, i)

    def plot_convex_hull(self, dimension: str):
        if len(self.dimensions) > 1:
            raise Exception("plotting multidimensional sweeps isn't implemented yet because then you have to\
                choose a value for the other dimensions and i don't feel like setting that up rn sorry - matt")

        d = None
        for dim in self.dimensions:
            if dim[0] == dimension:
                d = dim
                break
        if d is None:
            raise Exception("dimension not found")

        fig = plt.figure()
        subplot = fig.add_subplot()

        z_min = min(d[1])
        z_max = max(d[1])
        for i, res in enumerate(self.results):
            x, y = res[6]
            x = list(x) + [x[0]]  # wrap around
            y = list(y) + [y[0]]
            z = d[1][i]
            c = (z-z_min) / (z_max-z_min)
            subplot.plot(x, y, label=f'{d[0]} = {round(z, 3)}', color=(c, 0, 1-c))

        subplot.set_xlabel('Lateral Acceleration (m/s^2)')
        subplot.set_ylabel('Yaw Acceleration (rad/s^2)')
        plt.axhline(c="gray", linewidth=0.5)
        plt.axvline(c="gray", linewidth=0.5)
        subplot.legend()

        plt.show()
