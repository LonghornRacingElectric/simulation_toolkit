from sim.model_parameters.cars.car import Car
from sim.model_parameters.drivers.driver import Driver
from sim.model_parameters.telemetry.telemetry import Telemetry
from sim.model_parameters.vcu.vcu import VehicleControlUnit
from sim.system_models.aux_systems.controls_mux import ControlsMux
from sim.system_models.aux_systems.driver_model import DriverModel
from sim.system_models.aux_systems.telemetry_model import TelemetryModel
from sim.system_models.aux_systems.time_integrator import TimeIntegrator
from sim.system_models.aux_systems.vcu_model import VehicleControlUnitModel
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.vehicle_model import VehicleModel

import matplotlib.pyplot as plt
import numpy as np
import copy


class TransientSimulation:
    def __init__(self, duration: float, time_step: float, car: Car, driver: Driver,
                 telemetry: Telemetry, vcu: VehicleControlUnit):
        self.vehicle = VehicleModel(car)
        self.driver = DriverModel(driver)
        self.telemetry = TelemetryModel(telemetry)
        self.vcu = VehicleControlUnitModel(car, vcu)
        self.controls_mux = ControlsMux()
        self.time_integrator = TimeIntegrator(time_step, car)

        self.duration = duration
        self.time_step = time_step

        self.data = []

    def _get_initial_state(self, vehicle: VehicleModel) -> StateVector:
        state = StateVector()
        state.hv_battery_charge = vehicle.vehicle_parameters.hv_battery_capacity
        state.lv_battery_charge = vehicle.vehicle_parameters.lv_battery_capacity
        return state

    def run(self):
        state = self._get_initial_state(self.vehicle)
        state_dot = StateDotVector()
        time = 0

        while time < self.duration:
            driver_controls = self.driver.eval(time, state, state_dot)
            sensor_data = self.telemetry.eval(state, state_dot, driver_controls)
            vcu_output = self.vcu.eval(sensor_data)
            controls = self.controls_mux.eval(driver_controls, vcu_output)
            state_dot, observables = self.vehicle.eval(controls, state)
            state = self.time_integrator.eval(state, state_dot)

            time += self.time_step

            self.data.append((time, copy.copy(state), copy.copy(state_dot), driver_controls,
                              sensor_data, vcu_output, controls, observables))

            if driver_controls.e_stop:
                break

    def _plot(self, i: int, name: str):
        x = np.array([t[0] for t in self.data])
        y = np.array([getattr(t[i], name) for t in self.data])
        plt.scatter(x, y)
        plt.plot(x, y)
        plt.show()
        pass

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
