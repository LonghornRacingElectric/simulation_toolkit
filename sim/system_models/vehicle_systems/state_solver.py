from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vehicle_systems.suspension_model import SuspensionModel
from sim.system_models.vehicle_systems.aero_model import AeroModel

import numpy as np


class StateSolver:
    def __init__(self):
        # Initialize necessary models
        self.suspension = SuspensionModel()
        self.aero = AeroModel()

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_vector: StateVector,
            state_dot_vector: StateDotVector, observables_vector: ObservablesVector) -> None:

        # Initialize heave, pitch, roll, long accel, lat accel, yaw accel, steered angle, body slip, velocity
        long_accel = state_vector.long_accel
        lat_accel = state_vector.lateral_accel
        yaw_accel = state_vector.yaw_accel

        # Initialize translational acceleration
        translational_accelerations_IMF = np.array([long_accel, lat_accel, 0])

        # Evaluate suspension
        self.suspension.eval(vehicle_parameters, controls_vector, state_vector, state_dot_vector, observables_vector)

        if state_vector.aero == False:
            aero_forces = 0
            aero_moments = 0
        else:
            self.aero.eval(vehicle_parameters, controls_vector, state_vector, state_dot_vector, observables_vector)

            aero_forces = np.array(observables_vector.aero_forces)
            aero_moments = np.array(observables_vector.aero_moments)
        
        # Sum vehicle centric forces and moments
        sus_forces = np.array([x + y + z + w for x, y, z, w in zip(*observables_vector.tire_forces_IMF)])
        sus_moments = np.array([x + y + z + w for x, y, z, w in zip(*observables_vector.tire_moments_IMF)])

        gravity_forces = np.array([0, 0, -vehicle_parameters.total_mass * vehicle_parameters.accel_gravity])

        total_forces = aero_forces + sus_forces + gravity_forces
        total_moments = aero_moments + sus_moments

        # Force and moment balance
        m_a = vehicle_parameters.total_mass * translational_accelerations_IMF
        force_residuals = m_a - total_forces

        I_alpha = np.dot(vehicle_parameters.sprung_inertia, np.array([0, 0, yaw_accel]))
        moment_residuals = I_alpha - total_moments

        # Log force and moment residuals
        observables_vector.summation_forces = force_residuals
        observables_vector.summation_moments = moment_residuals
        observables_vector.axle_residuals = [0, 0, 0, 0]