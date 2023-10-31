from sim.model_parameters.cars.car import Car
from sim.model_parameters.drivers.driver import Driver
from sim.model_parameters.telemetry.telemetry import Telemetry
from sim.model_parameters.vcu.vcu import VehicleControlUnit
from sim.system_models.aux_systems.controls_mux import ControlsMux
from sim.system_models.aux_systems.driver_model import DriverModel
from sim.system_models.aux_systems.telemetry_model import TelemetryModel
from sim.system_models.aux_systems.vcu_model import VehicleControlUnitModel
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.vehicle_model import VehicleModel

import matplotlib.pyplot as plt
import numpy as np
import copy
import time


class TransientSimulation:
    def __init__(self, duration: float, time_step: float, car: Car, driver: Driver,
                 telemetry: Telemetry, vcu: VehicleControlUnit):
        self.vehicle = VehicleModel(car)
        self.driver = DriverModel(driver)
        self.telemetry = TelemetryModel(telemetry)
        self.vcu = VehicleControlUnitModel(car, vcu)
        self.controls_mux = ControlsMux()

        self.duration = duration
        self.time_step = time_step
        self.sim_time = 0

        self.data = []
        self.key_points = []

    def _get_initial_state(self, vehicle: VehicleModel) -> StateVector:
        state = StateVector()
        state.hv_battery_charge = vehicle.vehicle_parameters.hv_battery_capacity
        state.lv_battery_charge = vehicle.vehicle_parameters.lv_battery_capacity
        return state

    def run(self):
        state = self._get_initial_state(self.vehicle)
        state_dot = StateDotVector()

        self.sim_time = 0
        cpu_start_time = time.time()

        try:

            while self.sim_time < self.duration:
                cpu_elapsed_time = round(time.time() - cpu_start_time, 2)
                print(f"\rSimulation Progress: {round(self.sim_time / self.duration * 100, 1)}%\t({cpu_elapsed_time}s elapsed)",
                      end='')

                driver_controls = self.driver.eval(self.sim_time, state, state_dot)
                sensor_data = self.telemetry.eval(state, state_dot, driver_controls)
                vcu_output = self.vcu.eval(sensor_data)
                controls = self.controls_mux.eval(driver_controls, vcu_output)
                state, state_dot, observables = self.vehicle.eval(controls, state, self.time_step)

                self.sim_time += self.time_step

                self.data.append((self.sim_time, copy.deepcopy(state), copy.deepcopy(state_dot), driver_controls,
                                  sensor_data, vcu_output, controls, observables))

                if driver_controls.e_stop:
                    break

        except Exception as e:
            print(e)

        finally:
            self.vcu.terminate_subprocess()

        self._calculate_key_points()

        cpu_elapsed_time = round(time.time() - cpu_start_time, 2)
        print(f"\rSimulation Complete ({cpu_elapsed_time}s)")

    def _plot(self, i: int, name: str):
        x = np.array([t[0] for t in self.data])
        y = np.array([getattr(t[i], name) for t in self.data])
        is_multiple = len(y.shape) > 1

        ys = [y]
        if is_multiple:
            ys = y.T

        for i, y in enumerate(ys):
            plt.scatter(x, y, s=0.5)
            plt.plot(x, y, label=f"[{i}]")
        plt.title(name)
        plt.xlabel("time (s)")
        plt.ylabel(name)
        if is_multiple:
            plt.legend()
        plt.show()

    def plot_state(self, name: str):
        self._plot(1, name)

    def plot_state_dot(self, name: str):
        self._plot(2, name)

    def plot_driver_control(self, name: str):
        self._plot(3, name)

    def plot_sensor(self, name: str):
        self._plot(4, name)

    def plot_vcu_output(self, name: str):
        self._plot(5, name)

    def plot_vehicle_control(self, name: str):
        self._plot(6, name)

    def plot_observable(self, name: str):
        self._plot(7, name)

    def plot_map(self):
        t = np.array([t[0] for t in self.data])
        xyz = [t[1].displacement for t in self.data]
        x, y = np.array([v[0] for v in xyz]), np.array([v[1] for v in xyz])

        padding = 10
        x_min = min(x) - padding
        x_max = max(x) + padding
        y_min = min(y) - padding
        y_max = max(y) + padding

        plt.scatter(x, y, c=[(0.5-np.sin(c*2*np.pi)/2, 0, 0.5+np.sin(c*2*np.pi)/2) for c in t], s=0.5)
        # plt.plot(x, y)
        plt.title("displacement")
        plt.xlabel("x pos (m)")
        plt.ylabel("y pos (m)")
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)
        plt.grid(color='gray', linestyle='-', linewidth=0.5)
        plt.show()


    def _calculate_key_points(self):
        self.key_points = [
            ("planned duration", float(self.duration), "s"),
            ("actual duration", self.sim_time, "s"),
        ]

    def print_key_points(self):
        for label, value, units in self.key_points:
            print(f'{label.ljust(25)} | {str(round(value, 3)).ljust(8)} {units}')
