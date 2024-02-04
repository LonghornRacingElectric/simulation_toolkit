import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial import ConvexHull
import time

from sim.model_parameters.cars.car import Car
from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.aux_systems.steady_state_solver import SteadyStateSolver
from sim.util.math.conversions import *


class MmmSolver:

    def __init__(self, car: Car, mesh: int = 21, velocity: float = 25.0):
        self.done = False
        self.new_model = SteadyStateSolver()
        self.test_car = car
        self.test_controls_vector = ControlsVector()
        self.test_state_vector = StateVector()
        self.test_state_dot_vector = StateDotVector()
        self.test_observables_vector = ObservablesVector()

        self.mesh = mesh
        self.velocity = velocity

        self.steered_angle_iso_lines = []
        self.body_slip_iso_lines = []
        self.all_points = []

        self.linear_control = 0
        self.linear_stability = 0
        self.max_lat_accel = 0
        self.limit_yaw_stability = 0
        self.max_yaw_accel = 0
        self.trim_lat_accel = 0

    def _vehicle_model(self, x, y):
        # Prescribed values
        self.test_state_vector.body_slip = y[0]
        self.test_state_vector.speed = y[1]
        self.test_controls_vector.steering_angle = y[2]

        # Iterated values
        self.test_observables_vector.long_accel = x[0]
        self.test_observables_vector.lateral_accel = x[1]
        self.test_observables_vector.yaw_accel = x[2]
        self.test_state_vector.heave = x[3]
        self.test_state_vector.pitch = x[4]
        self.test_state_vector.roll = x[5]
        self.test_state_vector.wheel_slip_ratios = np.array([0, 0, 0, 0])

        self.new_model.eval(vehicle_parameters=self.test_car, controls_vector=self.test_controls_vector,
                            state_vector=self.test_state_vector,
                            state_dot_vector=self.test_state_dot_vector,
                            observables_vector=self.test_observables_vector)

        return [*self.test_observables_vector.summation_forces, *self.test_observables_vector.summation_moments]

    def solve(self):
        body_slip_sweep = np.linspace(deg_to_rad(-15), deg_to_rad(15), self.mesh)
        steered_angle_sweep = np.linspace(deg_to_rad(-15 * 3.5), deg_to_rad(15 * 3.5), self.mesh)

        self.body_slip_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.steered_angle_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.all_points = []

        counter = 0
        start_time = time.time()
        for i, body_slip in enumerate(body_slip_sweep):
            for j, steered_angle in enumerate(steered_angle_sweep):
                for velocity in [self.velocity]:
                    elapsed_time = round(time.time() - start_time, 2)
                    print(f"\rMMM Progress: {round(counter / self.mesh**2 * 100, 1)}%\t({elapsed_time}s elapsed)",
                          end='')
                    counter += 1

                    def solve_attempt(x):
                        return self._vehicle_model(x, [body_slip, velocity, steered_angle])

                    try:
                        fsolve_results: list[int] = list(fsolve(solve_attempt, np.array([0, 0, 0, 0, 0, 0])))
                    except:
                        continue

                    lat_accel = fsolve_results[1]
                    yaw_accel = fsolve_results[2]

                    self.steered_angle_iso_lines[j][0] = self.test_observables_vector.average_steered_angle
                    self.steered_angle_iso_lines[i][1][j] = lat_accel
                    self.steered_angle_iso_lines[i][2][j] = yaw_accel
                    self.body_slip_iso_lines[i][0] = body_slip
                    self.body_slip_iso_lines[j][1][i] = lat_accel
                    self.body_slip_iso_lines[j][2][i] = yaw_accel
                    self.all_points.append((lat_accel, yaw_accel))

        elapsed_time = round(time.time() - start_time, 2)
        print(f"\rMMM Complete ({elapsed_time}s)")

        self._calculate_key_points()

        self.done = True

    def plot(self):
        if not self.done:
            raise Exception("can't plot the MMM before you solve the MMM bruh")

        plt.title("Yaw Acceleration vs Lateral Acceleration")
        plt.xlabel("Lateral Acceleration (m/s^2)")
        plt.ylabel("Yaw Acceleration (rad/s^2)")
        plt.axhline(c="gray", linewidth=0.5)
        plt.axvline(c="gray", linewidth=0.5)

        mp = self.mesh // 2

        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="red", linewidth=0.8)
            plt.scatter(lat_accels, yaw_accels, s=0.5, c="black")

            # text_pos = ((lat_accels[mp] + lat_accels[mp - 1]) / 2, (yaw_accels[mp] + yaw_accels[mp - 1]) / 2 - 1.0)
            text_pos = (lat_accels[-1] + ((lat_accels[-1] < 0) * (lat_accels[-1] * 0.05 - 0.5)), yaw_accels[-1] + 0.7)
            plt.text(text_pos[0], text_pos[1], f'δ = {round(rad_to_deg(steered_angle), 1)}°', fontsize=6, c="red")

        for body_slip, lat_accels, yaw_accels in self.body_slip_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="blue", linewidth=0.8)
            a = 0.5 + np.sin(lat_accels[mp] ** 3 / 500) * 0.4
            text_pos = (lat_accels[mp] * a + lat_accels[mp + 1] * (1-a), yaw_accels[mp] * a + yaw_accels[mp + 1] * (1-a) + 0.2)
            # text_pos = (lat_accels[0] + 0.6, yaw_accels[0] + 0.3)
            plt.text(text_pos[0], text_pos[1], f'β = {round(rad_to_deg(body_slip), 1)}°', fontsize=6, c="blue")

        plt.show()

    def _calculate_key_points(self):
        mp = self.mesh // 2

        # delta (yaw accel) / delta (average tire steered angle)
        self.linear_control = self.body_slip_iso_lines[mp][2][mp + 1] / self.steered_angle_iso_lines[mp + 1][0]

        # delta (yaw accel) / delta (body slip angle)
        self.linear_stability = self.steered_angle_iso_lines[mp][2][mp + 1] / self.body_slip_iso_lines[mp + 1][0]

        self.max_lat_accel = 0
        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            for i in range(len(lat_accels)):
                if lat_accels[i] > self.max_lat_accel:
                    self.max_lat_accel = lat_accels[i]
                    self.limit_yaw_stability = yaw_accels[i]

        self.max_yaw_accel = max([max(x[2]) for x in self.steered_angle_iso_lines])

        self.trim_lat_accel = 0
        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            lat = np.interp(0, yaw_accels, lat_accels)
            if lat > self.trim_lat_accel:
                self.trim_lat_accel = lat

        self.all_points = np.array(self.all_points)
        convex_hull = ConvexHull(self.all_points)
        self.hull = self.all_points[convex_hull.vertices, 0], self.all_points[convex_hull.vertices, 1]

        self.key_points = [
            ("linear control at β=0", self.linear_control, "(rad/s^2)/rad"),
            ("linear stability at δ=0", self.linear_stability, "(rad/s^2)/rad"),
            ("max lat accel", self.max_lat_accel, "m/s^2"),
            ("yaw accel at max lat", self.limit_yaw_stability, "rad/s^2"),
            ("max yaw accel", self.max_yaw_accel, "rad/s^2"),
            ("trim lat accel", self.trim_lat_accel, "m/s^2"),
            (self.hull[0], self.hull[1])
        ]

    def print_key_points(self):
        for label, value, units in self.key_points[:6]:
            print(f'{label.ljust(25)} | {str(round(value, 3)).ljust(8)} {units}')
