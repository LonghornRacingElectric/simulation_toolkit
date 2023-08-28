from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vehicle_systems.tire_model import TireModel
from sim.system_models.vehicle_systems.aero_model import AeroModel
from sim.util.math import coords

import numpy as np

from scipy.interpolate import griddata
import pandas as pd
suspension_df = pd.read_csv("../../../data/parameter_curves/car/suspension_response_surfaces/FL_heave_roll_steer_camber.csv")


# All coords with respect to SAE J670
class SuspensionModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        # Possibly add aero here
        self.tires = [TireModel(), TireModel(), TireModel(), TireModel()]  # FL, FR, RL, RR
        self.aero = AeroModel()

        # Haven't added 4 DOF for axles. Accel/brake pedal will come into play here
        self.controls_in = [
            "steering_angle"
        ]

        self.state_in = [
            "heave",
            "pitch",
            "roll",
            "vehicle_velocity",
            "body_slip",
            "lateral_accel"
        ]

        self.state_out = [
        ]

        self.observables_out = [
            "vehicle_FX",
            "vehicle_FY",
            "vehicle_FZ",
            "vehicle_MX",
            "vehicle_MY",
            "vehicle_MZ"
        ]

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_vector: StateVector,
             state_dot_vector: StateDotVector, observables_vector: ObservablesVector):

        # Initialize tire coefficients
        front_FY_coeffs = vehicle_parameters.front_tire_coeff_Fy
        front_FX_coeffs = vehicle_parameters.front_tire_coeff_Fx
        rear_FY_coeffs = vehicle_parameters.rear_tire_coeff_Fy
        rear_FX_coeffs = vehicle_parameters.rear_tire_coeff_Fx

        coeff_array = [(front_FX_coeffs, front_FY_coeffs), (front_FX_coeffs, front_FY_coeffs),
                       (rear_FX_coeffs, rear_FY_coeffs), (rear_FX_coeffs, rear_FY_coeffs)]
        
        # Set tire coefficients
        for i in range(len(coeff_array)):
            self.tires[i].long_coeffs = coeff_array[i][0]
            self.tires[i].lat_coeffs = coeff_array[i][1]

        # Initialize heave, pitch, and roll
        chassis_heave = state_vector.heave
        chassis_pitch = state_vector.pitch
        chassis_roll = state_vector.roll

        # Initialize slip angle params
        steering_wheel_angle = controls_vector.steering_angle
        toe_angles = vehicle_parameters.toe_angles
        tire_positions = vehicle_parameters.tire_positions
        velocity = state_vector.velocity
        body_slip = state_vector.body_slip

        # Initialize inclination angle params
        static_IAs = vehicle_parameters.static_IAs
        roll_IA_gain = vehicle_parameters.roll_IA_gain
        heave_IA_gain = vehicle_parameters.heave_IA_gain

        # Initialize translational acceleration
        translational_accelerations_IMF = np.array([state_vector.long_accel, state_vector.lateral_accel, 0])
        translational_accelerations_NTB = np.matmul(coords.rotation_z(body_slip), translational_accelerations_IMF)

        # Secondary slip angle calculations
        steered_angles = np.array(self._get_steered_angles(vehicle_parameters = vehicle_parameters,
                                                           steered_angle = steering_wheel_angle))
        toe_angles = np.array(toe_angles)
        adjusted_steering_angles = steered_angles + toe_angles
        IMF_velocity = self._get_IMF_vel(velocity = velocity, 
                                         body_slip = body_slip)
        yaw_rate = self._get_yaw_vel(lateral_accel = float(translational_accelerations_NTB[1]),
                                     vehicle_velocity = IMF_velocity,
                                     body_slip = body_slip)

        # Log average steered angle for isolines
        observables_vector.average_steered_angle = (steered_angles[0] + steered_angles[1]) / 2

        # Calculate normal loads, slip angles, and inclination angles for each tire [Fl, FR, RL, RR]
        normal_loads = self._get_FZ_decoupled(vehicle_parameters = vehicle_parameters,
                                              heave = chassis_heave,
                                              pitch = chassis_pitch,
                                              roll = chassis_roll)

        slip_angles = self._get_SA(vehicle_velocity = IMF_velocity,
                                   steering_angles = steered_angles,
                                   toe_angles = toe_angles,
                                   tire_position = tire_positions,
                                   yaw_rate = yaw_rate)

        inclination_angles = self._get_IA(vehicle_parameters = vehicle_parameters, 
                                          adj_steering = steered_angles,
                                          heave = chassis_heave, 
                                          pitch = chassis_pitch, 
                                          roll = chassis_roll, 
                                          static_IA = static_IAs)
        
        # Log normal loads, slip angles, and inclination angles
        observables_vector.normal_loads = normal_loads
        observables_vector.slip_angles = slip_angles
        observables_vector.inclination_angles = inclination_angles

        # Get tire output
        tire_outputs = []
        for i in range(len(self.tires)):
            tire_forces = self.tires[i]._get_comstock_forces(SR = 0,
                                                             SA = slip_angles[i],
                                                             FZ = normal_loads[i],
                                                             IA = inclination_angles[i])
            
            tire_outputs.append([round(x) for x in tire_forces])

            # Rotate tire forces to correspond with true steered angle (now in terms of force on the vehicle)
            vehicle_centric_forces = np.matmul(coords.rotation_z(adjusted_steering_angles[i]), tire_forces)

            # Calculate vehicle moments due to tire forces
            vehicle_centric_moments = np.cross(tire_positions[i], vehicle_centric_forces)

            # Log force and moment calculations from above
            observables_vector.tire_forces_IMF[i] = vehicle_centric_forces
            observables_vector.tire_moments_IMF[i] = vehicle_centric_moments

        # Log direct tire model outputs
        observables_vector.tire_model_force_outputs = tire_outputs

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

        I_alpha = np.dot(vehicle_parameters.sprung_inertia, np.array([0, 0, state_vector.yaw_accel]))
        moment_residuals = I_alpha - total_moments

        # Log force and moment residuals
        observables_vector.summation_forces = force_residuals
        observables_vector.summation_moments = moment_residuals

    # Top level functions
    def _get_FZ_decoupled(self, vehicle_parameters: Car, heave: float, pitch: float, roll: float) -> list[float]:

        # Heave contribution
        F_heave_rate_spring = vehicle_parameters.front_heave_springrate / vehicle_parameters.front_heave_MR ** 2
        F_heave_rate_tire = vehicle_parameters.front_tire_vertical_rate
        F_heave_rate = F_heave_rate_spring * (2 * F_heave_rate_tire) / (F_heave_rate_spring + (2 * F_heave_rate_tire))

        R_heave_rate_spring = vehicle_parameters.rear_heave_springrate / vehicle_parameters.rear_heave_MR ** 2
        R_heave_rate_tire = vehicle_parameters.rear_tire_vertical_rate
        R_heave_rate = R_heave_rate_spring * (2 * R_heave_rate_tire) / (R_heave_rate_spring + (2 * R_heave_rate_tire))

        front_heave = heave
        rear_heave = heave

        # Pitch contribution
        front_track_to_CG = vehicle_parameters.wheelbase * vehicle_parameters.cg_bias
        rear_track_to_CG = vehicle_parameters.wheelbase * (1 - vehicle_parameters.cg_bias)

        front_pitch_displacement = front_track_to_CG * np.tan(abs(pitch)) * (1 if pitch > 0 else -1)
        rear_pitch_displacement = rear_track_to_CG * np.tan(abs(pitch)) * (1 if pitch < 0 else -1)

        F_adjusted_heave = front_heave + front_pitch_displacement
        R_adjusted_heave = rear_heave + rear_pitch_displacement

        F_force_heave = F_heave_rate * F_adjusted_heave
        R_force_heave = R_heave_rate * R_adjusted_heave

        # Roll contribution
        F_roll_rate_spring = 1 / 2 * vehicle_parameters.front_track ** 2 * (
                    (vehicle_parameters.front_roll_springrate / vehicle_parameters.front_roll_MR ** 2) / 2)
        F_roll_rate_tire = 1 / 2 * vehicle_parameters.front_track ** 2 * vehicle_parameters.front_tire_vertical_rate
        F_roll_rate = F_roll_rate_spring * F_roll_rate_tire / (F_roll_rate_spring + F_roll_rate_tire)

        R_roll_rate_spring = 1 / 2 * vehicle_parameters.rear_track ** 2 * (
                    (vehicle_parameters.rear_roll_springrate / vehicle_parameters.rear_roll_MR ** 2) / 2)
        R_roll_rate_tire = 1 / 2 * vehicle_parameters.rear_track ** 2 * vehicle_parameters.rear_tire_vertical_rate
        R_roll_rate = R_roll_rate_spring * R_roll_rate_tire / (R_roll_rate_spring + R_roll_rate_tire)

        # roll_moment = tire_force * track_width -> tire_force = roll_moment / track_width
        F_roll_moment = F_roll_rate * abs(roll)
        R_roll_moment = R_roll_rate * abs(roll)

        F_roll_tire_force = F_roll_moment / vehicle_parameters.front_track
        R_roll_tire_force = R_roll_moment / vehicle_parameters.rear_track

        # Load transfers
        FL_delta = F_roll_tire_force * (1 if roll < 0 else -1) + F_force_heave / 2
        FR_delta = F_roll_tire_force * (1 if roll > 0 else -1) + F_force_heave / 2
        RL_delta = R_roll_tire_force * (1 if roll < 0 else -1) + R_force_heave / 2
        RR_delta = R_roll_tire_force * (1 if roll > 0 else -1) + R_force_heave / 2

        # Static weights
        FL_static = vehicle_parameters.total_mass * vehicle_parameters.accel_gravity * (
                    1 - vehicle_parameters.cg_bias) / 2
        FR_static = vehicle_parameters.total_mass * vehicle_parameters.accel_gravity * (
                    1 - vehicle_parameters.cg_bias) / 2
        RL_static = vehicle_parameters.total_mass * vehicle_parameters.accel_gravity * vehicle_parameters.cg_bias / 2
        RR_static = vehicle_parameters.total_mass * vehicle_parameters.accel_gravity * vehicle_parameters.cg_bias / 2

        FL = FL_static + FL_delta
        FR = FR_static + FR_delta
        RL = RL_static + RL_delta
        RR = RR_static + RR_delta

        adjusted_FZ = [0 if x < 0 else x for x in [FL, FR, RL, RR]]

        return adjusted_FZ

    def _get_FZ_coupled(self, heave: float, pitch: float, roll: float) -> list[float]:
        return

    def _get_steered_angles(self, vehicle_parameters: Car, steered_angle: float):
        # TODO temporary linear regression, replace with CurveParameter
        outer_angle = 0.28166 * abs(steered_angle)  # - 0.00983
        inner_angle = 0.24888 * abs(steered_angle)  # + 0.0010

        if steered_angle == 0:
            return [0, 0, 0, 0]
        elif steered_angle > 0:
            return [inner_angle, outer_angle, 0, 0]
        else:
            return [-outer_angle, -inner_angle, 0, 0]

    def _get_SA(self, vehicle_velocity: list[float], steering_angles: list[float], toe_angles: list[float],
                tire_position: list[float], yaw_rate: float):

        tire_IMF_velocities = []
        for i in range(len(tire_position)):
            tire_velocity_IMF = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]), np.array(tire_position[i]))
            tire_IMF_velocities.append(tire_velocity_IMF)

        slip_angles = []
        for i in range(len(steering_angles)):
            slip_angle = (steering_angles[i] + toe_angles[i]) - np.arctan2(tire_IMF_velocities[i][1], tire_IMF_velocities[i][0])
            slip_angles.append(slip_angle)

        return slip_angles

    def _get_IA(self, vehicle_parameters: Car, adj_steering: list[float], heave: float, pitch: float, roll: float, 
                static_IA: list[float]) -> list[float]:

        # IAs = []
        # for i in range(len(adj_steering)):
        #     closest_steer = suspension_df.iloc[(suspension_df['steered_angle'] - adj_steering[i]).abs().argsort()[0]]['steered_angle']
        #     constant_steer_surface = suspension_df[(suspension_df['steered_angle'] == closest_steer)]
        #     heave_roll_pairs = [[x, y] for x, y in zip(list(constant_steer_surface['heave']), list(constant_steer_surface['adjusted_roll']))]
        #     camber_change_values = list(np.array(constant_steer_surface['camber_change']) * (1 if i == 0 or i == 2 else -1))
            
        #     IA_change = float(griddata(heave_roll_pairs, camber_change_values, [heave, roll], method = 'linear')[0])
        #     adjusted_IA = static_IA[i] + IA_change
        #     IAs.append(adjusted_IA)

        IAs = static_IA

        return IAs

    # Support the above functions
    def _get_yaw_vel(self, lateral_accel: float, vehicle_velocity: list[float], body_slip: float):
        # a_c = v^2 / r
        # omega = v / r -> r = v / omega
        # a_c = omega * v -> omega = a_c / v
        # omega = a_c / v
        if lateral_accel == 0:
            yaw_vel = 0
        
        else:
            yaw_vel = lateral_accel / np.linalg.norm(vehicle_velocity)

        return yaw_vel

    def _get_IMF_vel(self, velocity: float, body_slip: float):
        IMF_velocity = velocity * np.array([np.cos(body_slip), np.sin(body_slip), 0])

        return IMF_velocity