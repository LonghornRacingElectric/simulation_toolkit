import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial import ConvexHull
import pandas as pd
import time
import warnings
from mpl_toolkits.mplot3d import Axes3D

from sim.model_parameters.cars.car import Car
from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.aux_systems.steady_state_solver import SteadyStateSolver
from sim.util.math.conversions import *


class GGVGeneration:

    def __init__(self, car: Car, mesh: int = 21, velocity_range: list[int] = [5, 30], aero: bool = True):
        self.done = False
        self.new_model = SteadyStateSolver()
        self.test_car = car
        self.test_controls_vector = ControlsVector()
        self.test_state_vector = StateVector()
        self.test_state_dot_vector = StateDotVector()
        self.test_observables_vector = ObservablesVector()

        self.mesh = mesh
        self.velocity_range = velocity_range
        self.max_torque = 0

        self.lat_accels = []
        self.long_accels = []
        self.slip_ratios = []
        self.torque_requests = []

    def _vehicle_model(self, x, y):
        # Prescribed values
        self.test_state_vector.body_slip = y[0]
        self.test_state_vector.speed = y[1]
        self.test_controls_vector.steering_angle = y[2]
        self.test_controls_vector.torque_request = y[3] * self.max_torque
        self.test_controls_vector.brake_pct = max(-y[3], 0)

        # Iterated values
        self.test_observables_vector.long_accel = x[0]
        self.test_observables_vector.lateral_accel = x[1]
        self.test_observables_vector.yaw_accel = x[2]
        self.test_state_vector.heave = x[3]
        self.test_state_vector.pitch = x[4]
        self.test_state_vector.roll = x[5]
        self.test_state_vector.wheel_slip_ratios = np.array(x[6:])

        self.new_model.eval(vehicle_parameters=self.test_car, controls_vector=self.test_controls_vector,
                            state_vector=self.test_state_vector,
                            state_dot_vector=self.test_state_dot_vector,
                            observables_vector=self.test_observables_vector)

        residuals = [*self.test_observables_vector.summation_forces, *self.test_observables_vector.summation_moments, *self.test_observables_vector.axle_residuals]
        # print(*[round(r) for r in residuals])
        return residuals

    def solve(self):
        self.max_torque = self.test_car.max_torque
        
        body_slip_sweep = np.linspace(deg_to_rad(-15), deg_to_rad(15), self.mesh)
        steered_angle_sweep = np.linspace(deg_to_rad(-55), deg_to_rad(55), self.mesh)
        velocity_sweep = np.linspace(self.velocity_range[0], self.velocity_range[1], self.mesh)
        torque_request_sweep = np.linspace(-1, 1, self.mesh)

        self.lat_accels = []
        self.long_accels = []
        self.slip_ratios = []
        self.torque_requests = []
        self.heaves = []
        self.pitches = []
        self.rolls = []
        self.steered_angles = []
        self.tan_pitch = []
        self.velocities = []

        counter = 0
        start_time = time.time()
        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                for torque_request in torque_request_sweep:
                    for velocity in velocity_sweep:

                        elapsed_time = round(time.time() - start_time, 2)
                        print(f"\rGGV Progress: {round(counter / self.mesh**4 * 100, 1)}%\t({elapsed_time}s elapsed)",
                              end='')
                        counter += 1

                        if abs(torque_request) < 0.85 and (abs(steered_angle) < deg_to_rad(15) or abs(body_slip) < deg_to_rad(8)):
                            continue

                        def solve_attempt(x):
                            # print(*[round(a, 2) for a in x])
                            adjusted_SR = [max(-1, min(1, sr)) for sr in x[6:]]
                            return self._vehicle_model([*x[:6], *adjusted_SR],
                                                       [body_slip, velocity, steered_angle, torque_request])


                        # long accel, lat accel, yaw accel, heave, pitch, roll, FL SR, FR SR, RL SR, RR SR
                        with warnings.catch_warnings():
                            warnings.simplefilter("ignore")
                            # try:
                            fsolve_results: list[int] = list(fsolve(solve_attempt, np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])))
                            # except:
                            #     continue

                        self.long_accels.append(fsolve_results[0])
                        self.lat_accels.append(fsolve_results[1])
                        self.slip_ratios.append([max(-1, min(1, sr)) for sr in fsolve_results[6:]])
                        self.torque_requests.append(torque_request)
                        self.heaves.append(fsolve_results[3])
                        self.pitches.append(fsolve_results[4])
                        self.rolls.append(fsolve_results[5])
                        self.steered_angles.append(steered_angle)
                        self.tan_pitch.append(np.tan(fsolve_results[4]))
                        self.velocities.append(velocity)

        elapsed_time = round(time.time() - start_time, 2)
        print(f"\rGGV Complete ({elapsed_time}s)")

        # self._calculate_key_points()

        self.done = True

    def plot(self):
        if not self.done:
            raise Exception("can't plot the GGV before you solve for the GGV bruh")
        
        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        ax = plt.axes(projection='3d')

        ax.scatter3D([x / self.test_car.accel_gravity for x in self.lat_accels], [x / self.test_car.accel_gravity for x in self.long_accels], self.velocities, cmap='Greens', s = 10)

        ax.set_title("GGV bro")

        ax.set_xlabel('Lateral Accel (G)')
        ax.set_ylabel('Longitudinal Accel (G)')
        ax.set_zlabel('Velocity (m/s)')

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
        for label, value, units in self.key_points:
            print(f'{label.ljust(25)} | {str(round(value, 3)).ljust(8)} {units}')
