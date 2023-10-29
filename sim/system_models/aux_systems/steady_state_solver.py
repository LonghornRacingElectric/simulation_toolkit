from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vehicle_systems.suspension_model import SuspensionModel
from sim.system_models.vehicle_systems.aero_model import AeroModel
from sim.system_models.vehicle_systems.powertrain_model import PowertrainModel

import numpy as np

from sim.util.math.conversions import rads_to_rpm


class SteadyStateSolver:
    def __init__(self):
        # Initialize necessary models
        self.suspension = SuspensionModel()
        self.aero = AeroModel()
        self.powertrain = PowertrainModel()

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_vector: StateVector,
             state_dot_vector: StateDotVector, observables_vector: ObservablesVector) -> None:

        # Initialize heave, pitch, roll, long accel, lat accel, yaw accel, steered angle, body slip, velocity
        long_accel = observables_vector.long_accel
        lat_accel = observables_vector.lateral_accel
        yaw_accel = observables_vector.yaw_accel

        # Initialize translational acceleration
        translational_accelerations_IMF = np.array([long_accel, lat_accel, 0])

        # motor rpm
        state_vector.motor_rpm = 0  # rads_to_rpm(diff_angular_velocity * vehicle_parameters.gear_ratio)

        # Evaluate powertrain
        self.powertrain.eval(vehicle_parameters, controls_vector, state_vector, state_dot_vector, observables_vector)

        # Evaluate suspension
        self.suspension.eval(vehicle_parameters, controls_vector, state_vector, state_dot_vector, observables_vector)

        # Evaluate aerodynamics
        self.aero.eval(vehicle_parameters, controls_vector, state_vector, state_dot_vector, observables_vector)

        gravity_forces = np.array([0, 0, -vehicle_parameters.total_mass * vehicle_parameters.accel_gravity])
        total_forces = state_dot_vector.aero_forces + state_dot_vector.sus_forces + gravity_forces
        total_moments = state_dot_vector.aero_moments + state_dot_vector.sus_moments

        # Force and moment balance
        m_a = vehicle_parameters.total_mass * translational_accelerations_IMF
        force_residuals = m_a - total_forces

        I_alpha = np.dot(vehicle_parameters.sprung_inertia, np.array([0, 0, yaw_accel]))
        moment_residuals = I_alpha - total_moments

        # Axle equilibrium
        front_axle = state_dot_vector.tire_torques[:2] - state_dot_vector.powertrain_torques[:2]
        rear_axle = state_dot_vector.tire_torques[2:] - state_dot_vector.powertrain_torques[2:]
        axle_residuals = [*front_axle, *rear_axle]

        # Log force and moment residuals
        observables_vector.summation_forces = force_residuals
        observables_vector.summation_moments = moment_residuals
        observables_vector.axle_residuals = axle_residuals
