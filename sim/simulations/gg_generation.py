import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial import ConvexHull

from sim.model_parameters.cars.car import Car
from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.aux_systems.steady_state_solver import SteadyStateSolver
from sim.util.math.conversions import *


class GGGeneration:

    def __init__(self, car: Car, mesh: int = 21, velocity: float = 25.0, aero: bool = True):
        self.done = False
        self.new_model = SteadyStateSolver()
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
        self.test_state_vector.velocity = y[1]
        self.test_controls_vector.steering_angle = y[2]
        self.test_controls_vector.torque_request = y[3]

        # Iterated values
        self.test_state_vector.long_accel = x[0]
        self.test_state_vector.lateral_accel = x[1]
        self.test_state_vector.yaw_accel = x[2]
        self.test_state_vector.heave = x[3]
        self.test_state_vector.pitch = x[4]
        self.test_state_vector.roll = x[5]
        self.test_state_vector.FL_SR = x[6]
        self.test_state_vector.FR_SR = x[7]
        self.test_state_vector.RL_SR = x[8]
        self.test_state_vector.RL_SR = x[9]

        self.new_model.eval(vehicle_parameters=self.test_car, controls_vector=self.test_controls_vector,
                            state_vector=self.test_state_vector,
                            state_dot_vector=self.test_state_dot_vector,
                            observables_vector=self.test_observables_vector)
        
        return [*self.test_observables_vector.summation_forces, *self.test_observables_vector.summation_moments, *self.test_observables_vector.axle_residuals]

    def solve(self):
        body_slip_sweep = np.linspace(deg_to_rad(-15), deg_to_rad(15), self.mesh)
        steered_angle_sweep = np.linspace(deg_to_rad(-55), deg_to_rad(55), self.mesh)
        torque_request_sweep = np.linspace(-1, 1, self.mesh)

        self.lat_accels = []
        self.long_accels = []

        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                for torque_request in torque_request_sweep:
                    for velocity in [self.velocity]:
                        def solve_attempt(x):
                            return self._vehicle_model(x, [body_slip, velocity, steered_angle, torque_request])

                        # try:
                            # long accel, lat accel, yaw accel, heave, pitch, roll, FL SR, FR SR, RL SR, RR SR
                        fsolve_results: list[int] = list(fsolve(solve_attempt, np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])))
                        # except:
                        #     continue

                        self.lat_accels.append(fsolve_results[1])
                        self.long_accels.append(fsolve_results[0])

        self._calculate_key_points()

        self.done = True

    def plot(self):
        if not self.done:
            raise Exception("can't plot the MMM before you solve the MMM bruh")

        plt.title("Lateral Acceleration vs Longitudinal Acceleration")
        plt.xlabel("Lateral Acceleration (m/s^2)")
        plt.ylabel("Longitudinal Acceleration (m/s^2)")
        plt.scatter(self.lat_accels, self.long_accels)
        plt.axhline(c="gray", linewidth=0.5)
        plt.axvline(c="gray", linewidth=0.5)

        plt.show()

    def _calculate_key_points(self):
        min_lat = min(self.lat_accels)
        max_lat = max(self.lat_accels)
        min_long = min(self.long_accels)
        max_long = max(self.long_accels)

        self.key_points = [
            ("Minimum Lateral Acceleration", min_lat, "m/s^2"),
            ("Maximum Lateral Acceleration", max_lat, "m/s^2"),
            ("Minimum Longitudinal Acceleration", min_long, "m/s^2"),
            ("Maximum Longitudinal Acceleration", max_long, "m/s^2"),
        ]

    def print_key_points(self):
        for label, value, units in self.key_points[:6]:
            print(f'{label.ljust(25)} | {str(round(value, 3)).ljust(8)} {units}')
