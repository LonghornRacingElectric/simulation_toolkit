from sim.model_parameters.cars.car import Car
from sim.model_parameters.drivers.driver import Driver
from sim.model_parameters.telemetry.telemetry import Telemetry
from sim.model_parameters.vcu.vcu import VehicleControlUnit
from sim.system_models.aux_systems.driver_model import DriverModel
from sim.system_models.aux_systems.telemetry_model import TelemetryModel
from sim.system_models.aux_systems.time_integrator import TimeIntegrator
from sim.system_models.aux_systems.vcu_model import VehicleControlUnitModel
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.vehicle_model import VehicleModel


class TransientSimulation:
    def __init__(self, duration: float, time_step: float, car: Car, driver: Driver,
                 telemetry: Telemetry, vcu: VehicleControlUnit):
        self.vehicle = VehicleModel(car)
        self.driver = DriverModel(driver)
        self.telemetry = TelemetryModel(telemetry)
        self.vcu = VehicleControlUnitModel(car, vcu)
        self.time_integrator = TimeIntegrator(time_step)

        self.duration = duration
        self.time_step = time_step

        self.data = []

    def run(self):
        state = StateVector()  # TODO ensure initial state
        state_dot = StateDotVector()
        time = 0

        while time < self.duration:
            driver_controls = self.driver.eval(time, state, state_dot)
            sensor_data = self.telemetry.eval(state, state_dot)
            vehicle_controls = self.vcu.eval(driver_controls, sensor_data)
            state_dot = self.vehicle.eval(vehicle_controls, state)
            state = self.time_integrator.eval(state, state_dot)

            time += self.time_step

            self.data.append((time, state, state_dot, driver_controls, sensor_data, vehicle_controls))

            if driver_controls.e_stop:
                break

            print(f"[{round(time, 2)}]\t {state.velocity} m/s")

    def plot(self, variable: str):
        return
        # TODO produce a matplotlib plot of the given variable vs time
