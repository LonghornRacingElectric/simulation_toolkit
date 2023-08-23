import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fsolve

from sim.model_parameters.cars.lady_luck import LadyLuck
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.suspension_model import SuspensionModel
from sim.util.math.conversions import *


class MmmSolver:

    def __init__(self):
        self.done = False
        self.new_model = SuspensionModel()
        self.test_car = LadyLuck()
        self.test_controls_vector = ControlsVector()
        self.test_state_vector = StateVector()
        self.test_state_dot_vector = StateDotVector()
        self.test_observables_vector = ObservablesVector()
        self.lat_accels = []
        self.yaw_accels = []

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
        mesh = 21

        body_slip_sweep = np.linspace(deg_to_rad(-10), deg_to_rad(10), mesh)
        steered_angle_sweep = np.linspace(deg_to_rad(-90), deg_to_rad(90), mesh)

        for body_slip in body_slip_sweep:
            for steered_angle in steered_angle_sweep:
                for velocity in [25]:
                    def solve_attempt(x):
                        return self._vehicle_model(x, [body_slip, velocity, steered_angle])

                    try:
                        fsolve_results = fsolve(solve_attempt, np.array([0, 0, 0, 0, 0, 0]), maxfev=1000)
                    except:
                        continue

                    self.lat_accels.append(fsolve_results[1])
                    self.yaw_accels.append(fsolve_results[2])

        self.done = True

    def plot(self):
        if not self.done:
            raise Exception("can't plot the MMM before you solve the MMM bruh")
        plt.scatter(self.lat_accels, self.yaw_accels, s=0.5)
        plt.xlim(-25, 25)
        plt.ylim(-30, 30)
        plt.show()
