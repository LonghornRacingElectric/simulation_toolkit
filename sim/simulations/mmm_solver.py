import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fsolve

from sim.model_parameters.cars.car import Car
from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.suspension_model import SuspensionModel
from sim.util.math.conversions import *


class MmmSolver:

    def __init__(self, car: Car, mesh: int = 21, velocity: float = 25.0, aero: bool = True):
        self.done = False
        self.new_model = SuspensionModel()
        self.test_car = car
        self.test_controls_vector = ControlsVector()
        self.test_state_vector = StateVector()
        self.test_state_dot_vector = StateDotVector()
        self.test_observables_vector = ObservablesVector()

        self.mesh = mesh
        self.velocity = velocity
        self.test_state_vector.aero = aero
        self.steered_angle_iso_lines = []
        self.body_slip_iso_lines = []

    def _vehicle_model(self, x, y):
        # Prescribed values
        self.test_state_vector.body_slip = y[0]
        self.test_state_vector.velocity = y[1]
        self.test_controls_vector.steering_angle = y[2]

        # Iterated values
        self.test_state_vector.long_accel = x[0]
        self.test_state_vector.lateral_accel = x[1]
        self.test_state_vector.yaw_accel = x[2]
        self.test_state_vector.heave = x[3]
        self.test_state_vector.pitch = x[4]
        self.test_state_vector.roll = x[5]

        self.new_model.eval(vehicle_parameters=self.test_car, controls_vector=self.test_controls_vector,
                            state_vector=self.test_state_vector,
                            state_dot_vector=self.test_state_dot_vector,
                            observables_vector=self.test_observables_vector)

        return [*self.test_observables_vector.summation_forces, *self.test_observables_vector.summation_moments]

    def solve(self):
        body_slip_sweep = np.linspace(deg_to_rad(-7), deg_to_rad(7), self.mesh)
        steered_angle_sweep = np.linspace(deg_to_rad(-40), deg_to_rad(40), self.mesh)

        self.body_slip_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.steered_angle_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]

        for i, body_slip in enumerate(body_slip_sweep):
            for j, steered_angle in enumerate(steered_angle_sweep):
                for velocity in [self.velocity]:
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

        self.done = True

    def plot(self):
        if not self.done:
            raise Exception("can't plot the MMM before you solve the MMM bruh")

        plt.title("Yaw Acceleration vs Lateral Acceleration")
        plt.xlim(-25, 25)
        plt.ylim(-45, 45)
        plt.xlabel("Lateral Acceleration (m/s^2)")
        plt.ylabel("Yaw Acceleration (rad/s^2)")
        plt.axhline(c="gray", linewidth=0.5)
        plt.axvline(c="gray", linewidth=0.5)

        mp = self.mesh // 2

        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="red", linewidth=0.8)
            plt.scatter(lat_accels, yaw_accels, s=0.5, c="black")

            text_pos = ((lat_accels[mp] + lat_accels[mp - 1]) / 2 - 0.8, (yaw_accels[mp] + yaw_accels[mp - 1]) / 2 + 1.5)
            plt.text(text_pos[0], text_pos[1], f'δ = {round(rad_to_deg(steered_angle), 1)}°', fontsize=6, c="red")

        for body_slip, lat_accels, yaw_accels in self.body_slip_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="blue", linewidth=0.8)
            text_pos = ((lat_accels[mp] + lat_accels[mp + 1]) / 2 - 1.9, (yaw_accels[mp] + yaw_accels[mp + 1]) / 2)
            plt.text(text_pos[0], text_pos[1], f'β = {round(rad_to_deg(body_slip))}°', fontsize=6, c="blue")

        plt.show()

    def calc_linear_control(self):
        mp = self.mesh // 2
        # delta (yaw accel) / delta (average tire steered angle)
        return self.body_slip_iso_lines[mp][2][mp + 1] / self.steered_angle_iso_lines[mp + 1][0]

    def calc_linear_stability(self):
        mp = self.mesh // 2
        # delta (yaw accel) / delta (body slip angle)
        return self.steered_angle_iso_lines[mp][2][mp + 1] / self.body_slip_iso_lines[mp - 1][0]

    def calc_max_lat_accel(self):
        return 0  # TODO implement

    def calc_limit_yaw_stability(self):
        return 0  # TODO implement

    def calc_trim_lat_accel(self):
        return 0  # TODO implement

    def calc_max_yaw_accel(self):
        return 0  # TODO implement

    def print(self):
        info = [
            ("linear control at β=0", self.calc_linear_control()),
            ("linear stability at δ=0", self.calc_linear_stability()),
            ("max lat accel", self.calc_max_lat_accel()),
            ("limit yaw stability", self.calc_limit_yaw_stability()),
            ("trim lat accel", self.calc_trim_lat_accel()),
            ("max yaw accel", self.calc_max_yaw_accel()),
        ]

        for label, value in info:
            print(f'{label}:')
            print(f'\t{value}')

