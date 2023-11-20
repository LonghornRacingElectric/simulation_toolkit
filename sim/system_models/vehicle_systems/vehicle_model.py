"""

Plan for this file was originally to have all the vehicle systems run in parallel, but it seems like that
structure didn't really work out so we might go back to having a kind of tree with suspension being the root
and things like aero and powertrain being children.

"""
import numpy as np

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vehicle_systems.aero_model import AeroModel
from sim.system_models.vehicle_systems.powertrain_model import PowertrainModel
from sim.system_models.vehicle_systems.suspension_model import SuspensionModel
from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel
from sim.util.math.conversions import rads_to_rpm
from sim.util.math.coords import rotation_z


class VehicleModel:
    def __init__(self, vehicle_parameters: Car):
        self.vehicle_parameters = vehicle_parameters
        self.vehicle_system_models: list[VehicleSystemModel] = [
            PowertrainModel(),
            SuspensionModel(transient=True),
            AeroModel(),
        ]
        self.first_time = True

    def eval(self, controls: ControlsVector, state: StateVector,
             time_step: float) -> (StateVector, StateDotVector, ObservablesVector):
        state_dot: StateDotVector = StateDotVector()
        observables: ObservablesVector = ObservablesVector()

        for vehicle_system_model in self.vehicle_system_models:
            if self.first_time:
                controls.expect_get(vehicle_system_model.controls_in)
                state.expect_get(vehicle_system_model.state_in)
                state_dot.expect_set(vehicle_system_model.state_out)
                observables.expect_set(vehicle_system_model.observables_out)

            vehicle_system_model.eval(self.vehicle_parameters, controls, state, state_dot, observables)

            if self.first_time:
                controls.confirm_expectations()
                state.confirm_expectations()
                state_dot.confirm_expectations()
                observables.confirm_expectations()
                self.first_time = False

        self.integrate(self.vehicle_parameters, state, state_dot, observables, time_step)

        return state, state_dot, observables

    def integrate(self, car: Car, state: StateVector, state_dot: StateDotVector, observables: ObservablesVector,
                  time_step: float):
        state.hv_battery_charge -= state_dot.hv_battery_current * time_step
        state.lv_battery_charge -= state_dot.lv_battery_current * time_step

        state.hv_battery_temperature += (state_dot.hv_battery_net_heat
                                         * car.hv_battery_thermal_resistance * time_step)
        state.inverter_temperature += (state_dot.inverter_net_heat
                                       * car.inverter_thermal_resistance * time_step)
        state.motor_temperature += (state_dot.motor_net_heat
                                    * car.hv_battery_thermal_resistance * time_step)
        state.coolant_temperature += (state_dot.coolant_net_heat
                                      * car.hv_battery_thermal_resistance * time_step)

        weight_force = np.array([0, 0, -car.accel_gravity]) * car.total_mass
        total_forces = state_dot.sus_forces + state_dot.aero_forces + weight_force

        acceleration_ntb = total_forces / car.total_mass
        acceleration = rotation_z(state.yaw, acceleration_ntb)
        acceleration = np.multiply(acceleration, [1, 1, 0])  # TODO just removing Z for now
        state.displacement += state.velocity * time_step
        state.velocity += acceleration * time_step

        total_moments = state_dot.sus_moments + state_dot.aero_moments
        yaw_accel = total_moments[2] / car.sprung_inertia[2][2]
        state.yaw += state.yaw_rate * time_step
        state.yaw_rate += yaw_accel * time_step

        state.heave = 0  # state.displacement[2]
        state.pitch = acceleration_ntb[0] * -0.0015  # TODO value based on accel
        state.roll = acceleration_ntb[1] * 0.0015  # TODO value based on accel

        velocity_angle = np.arctan2(state.velocity[1], state.velocity[0])
        state.speed = np.linalg.norm(np.multiply(state.velocity, [1, 1, 0]))
        state.body_slip = velocity_angle - state.yaw if state.speed > 0.1 else 0

        acceleration_imf = rotation_z(state.body_slip, acceleration_ntb)
        observables.long_accel = acceleration_imf[0]
        observables.lateral_accel = acceleration_imf[1]

        wheel_torques = state_dot.powertrain_torques - state_dot.tire_torques
        # print("ptn torques", state_dot.powertrain_torques)
        # print("tire torques", state_dot.tire_torques)
        # print("wheel torques", wheel_torques)
        wheel_angular_accels = wheel_torques / np.array([car.front_unsprung_inertia, car.front_unsprung_inertia,
                                                         car.rear_unsprung_inertia, car.rear_unsprung_inertia])
        state.wheel_angular_displacements += state.wheel_angular_velocities * time_step
        state.wheel_angular_velocities += wheel_angular_accels * time_step
        state.wheel_angular_velocities = np.maximum(state.wheel_angular_velocities, np.array([0, 0, 0, 0]))

        tire_velocities = np.array(observables.tire_heading_velocities)
        rolling_wheel_angular_velocities = np.divide(tire_velocities, np.array(car.tire_radii))

        state.wheel_angular_velocities = np.where(np.array([False, False, True, True]),  # powered vs rolling
                                                  state.wheel_angular_velocities, rolling_wheel_angular_velocities)

        tire_surface_velocities = np.multiply(state.wheel_angular_velocities, np.array(car.tire_radii))
        state.wheel_slip_ratios = np.divide(tire_surface_velocities - tire_velocities, tire_velocities,
                                            out=np.zeros_like(tire_velocities),
                                            where=tire_velocities != 0)
        state.wheel_slip_ratios = np.where(state.wheel_slip_ratios < 10,
                                           state.wheel_slip_ratios, np.full_like(state.wheel_slip_ratios, 10))
        state.wheel_slip_ratios = np.where(state.wheel_slip_ratios > -1,
                                           state.wheel_slip_ratios, np.full_like(state.wheel_slip_ratios, -1))

        state.motor_rpm = rads_to_rpm(np.average(state.wheel_angular_velocities[2:]) * car.gear_ratio)

