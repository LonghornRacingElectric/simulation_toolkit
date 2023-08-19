from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vehicle_systems.tire_model import TireModel

import numpy as np
# RIP
from scipy.optimize import fsolve


# All coords with respect to SAE J670
class SuspensionModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        # Possibly add aero here
        self.tires = [TireModel(), TireModel(), TireModel(), TireModel()]  # FL, FR, RL, RR

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

        # Intermediate steps (or extra information)
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

        # Tires
        front_FY_coeffs = vehicle_parameters.front_tire_coeff_Fy
        front_FX_coeffs = vehicle_parameters.front_tire_coeff_Fx
        rear_FY_coeffs = vehicle_parameters.rear_tire_coeff_Fy
        rear_FX_coeffs = vehicle_parameters.rear_tire_coeff_Fx

        coeff_array = [(front_FY_coeffs, front_FX_coeffs), (front_FY_coeffs, front_FX_coeffs),
                       (rear_FY_coeffs, rear_FX_coeffs), (rear_FY_coeffs, rear_FX_coeffs)]

        # Heave, pitch, and roll
        vehicle_heave = state_vector.heave
        vehicle_pitch = state_vector.pitch
        vehicle_roll = state_vector.roll
        # Slip angle params
        steered_angles = self._get_steered_angles(vehicle_parameters, controls_vector.steering_angle)
        adj_steering_angles = self._get_adj_steering([steered_angles[0], steered_angles[1], 0, 0],
                                                     vehicle_parameters.toe_angles)
        toe_angles = vehicle_parameters.toe_angles
        yaw_rate = self._get_yaw_vel(state_vector.lateral_accel,
                                     self._get_IMF_vel(state_vector.velocity, state_vector.body_slip))
        vehicle_velocity = self._get_IMF_vel(state_vector.velocity, state_vector.body_slip)
        tire_positions = vehicle_parameters.tire_positions
        # Inclination angle params
        static_IAs = vehicle_parameters.static_IAs
        roll_IA_gain = vehicle_parameters.roll_IA_gain
        heave_IA_gain = vehicle_parameters.heave_IA_gain

        print(steered_angles)

        # Get FZ, SA, IA
        normal_loads = self._get_FZ_decoupled(vehicle_parameters=vehicle_parameters, heave=vehicle_heave,
                                              pitch=vehicle_pitch, roll=vehicle_roll)

        slip_angles = self._get_SA(vehicle_velocity=vehicle_velocity, adj_steering_angles=adj_steering_angles,
                                   toe_angles=toe_angles, tire_position=tire_positions, yaw_rate=yaw_rate)

        inclination_angles = self._get_IA(vehicle_parameters=vehicle_parameters, adj_steering=adj_steering_angles,
                                           heave=vehicle_heave, pitch=vehicle_pitch, roll=vehicle_roll,
                                           static_IA=static_IAs,
                                           roll_IA_gain=roll_IA_gain, heave_IA_gain=heave_IA_gain)

        print(f'Normal loads: {normal_loads}')
        print(f'Slip angles: {slip_angles}')
        print(f'Inclination angles: {inclination_angles}')

        # Set fit coeffs
        for i in range(len(coeff_array)):
            self.tires[i].lat_coeffs = coeff_array[i][0]
            self.tires[i].long_coeffs = coeff_array[i][1]

        # Get tire output
        for i in range(len(self.tires)):
            tire_forces = self.tires[i]._get_comstock_forces(SR=0, SA=slip_angles[i], FZ=normal_loads[i],
                                                             IA=inclination_angles[i])
            observables_vector.tire_forces[i] = tire_forces

        pass

    # Top level functions
    def _get_FZ_decoupled(self, vehicle_parameters: Car, heave: float, pitch: float, roll: float) -> list[float]:

        # Heave contribution
        F_heave_rate_spring = vehicle_parameters.front_heave_springrate / vehicle_parameters.front_heave_MR ** 2
        F_heave_rate_tire = vehicle_parameters.front_tire_vertical_rate * 2
        F_heave_rate = F_heave_rate_spring * F_heave_rate_tire / (F_heave_rate_spring + F_heave_rate_tire)

        R_heave_rate_spring = vehicle_parameters.rear_heave_springrate / vehicle_parameters.rear_heave_MR ** 2
        R_heave_rate_tire = vehicle_parameters.rear_tire_vertical_rate * 2
        R_heave_rate = R_heave_rate_spring * R_heave_rate_tire / (R_heave_rate_spring + R_heave_rate_tire)

        front_heave = heave
        rear_heave = heave

        # Pitch contribution

        """
        Need to do this about pitch center. Calculate vertical displacements at the front and rear axle.
        Correlate to force using front and rear heave rates
        """

        # I'm gonna do these about the CG right now.. I'll fix this later
        # I'm also starting to doubt this method of superposition. It seemed reasonable at roll and heave, but
        # pitch seems very suspect

        front_track_to_CG = vehicle_parameters.wheelbase * vehicle_parameters.cg_bias
        rear_track_to_CG = vehicle_parameters.wheelbase * (1 - vehicle_parameters.cg_bias)

        front_pitch_displacement = front_track_to_CG * np.tan(pitch) * (1 if pitch > 0 else -1)
        rear_pitch_displacement = rear_track_to_CG * np.tan(pitch) * (1 if pitch < 0 else -1)

        F_adjusted_heave = front_heave + front_pitch_displacement
        R_adjusted_heave = rear_heave + rear_pitch_displacement

        F_force_heave = F_heave_rate * F_adjusted_heave
        R_force_heave = R_heave_rate * R_adjusted_heave

        # Roll contribution
        F_roll_rate_spring = 1 / 2 * vehicle_parameters.front_track ** 2 * (
                    vehicle_parameters.front_roll_springrate / vehicle_parameters.front_roll_MR ** 2)
        F_roll_rate_tire = 1 / 2 * vehicle_parameters.front_track ** 2 * vehicle_parameters.front_tire_vertical_rate
        F_roll_rate = F_roll_rate_spring * F_roll_rate_tire / (F_roll_rate_spring + F_roll_rate_tire)

        R_roll_rate_spring = 1 / 2 * vehicle_parameters.rear_track ** 2 * (
                    vehicle_parameters.rear_roll_springrate / vehicle_parameters.rear_roll_MR ** 2)
        R_roll_rate_tire = 1 / 2 * vehicle_parameters.rear_track ** 2 * vehicle_parameters.rear_tire_vertical_rate
        R_roll_rate = R_roll_rate_spring * R_roll_rate_tire / (R_roll_rate_spring + R_roll_rate_tire)

        # roll_moment = tire_force * track_width -> tire_force = roll_moment / track_width
        F_roll_moment = F_roll_rate * roll
        R_roll_moment = R_roll_rate * roll

        F_roll_tire_force = F_roll_moment / vehicle_parameters.front_track
        R_roll_tire_force = R_roll_moment / vehicle_parameters.rear_track

        # Load transfers
        FL_delta = -F_roll_tire_force / 2 + F_force_heave / 2
        FR_delta = F_roll_tire_force / 2 + F_force_heave / 2
        RL_delta = -R_roll_tire_force / 2 + R_force_heave / 2
        RR_delta = R_roll_tire_force / 2 + R_force_heave / 2

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

        return [FL, FR, RL, RR]

    def _get_FZ_coupled(self, heave: float, pitch: float, roll: float) -> list[float]:
        return

    def _get_steered_angles(self, vehicle_parameters: Car, steered_angle: float):
        outer_angle = 0.28166 * steered_angle - 0.00983
        inner_angle = 0.24888 * steered_angle + 0.0010

        if steered_angle == 0:
            return [0, 0]
        elif steered_angle > 0:
            return [inner_angle, outer_angle]
        else:
            return [outer_angle, inner_angle]

    def _get_SA(self, vehicle_velocity: list[float], adj_steering_angles: list[float], toe_angles: list[float],
                tire_position: list[float], yaw_rate: float):

        tire_IMF_velocities = []
        for i in range(len(tire_position)):
            tire_velocity_IMF = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]), tire_position[i])
            tire_IMF_velocities.append(tire_velocity_IMF)

        slip_angles = []
        for i in range(len(adj_steering_angles)):
            slip_angle = adj_steering_angles[i] - np.arctan2(tire_IMF_velocities[i][1], tire_IMF_velocities[i][0])
            slip_angles.append(slip_angle)

        return slip_angles

    def _get_IA(self, vehicle_parameters: Car, adj_steering: list[float], heave: float, pitch: float, roll: float,
                 static_IA: list[float], roll_IA_gain: list[float], heave_IA_gain: list[float]) -> list[float]:

        front_track_to_CG = vehicle_parameters.wheelbase * vehicle_parameters.cg_bias
        rear_track_to_CG = vehicle_parameters.wheelbase * (1 - vehicle_parameters.cg_bias)

        front_pitch_displacement = front_track_to_CG * np.tan(pitch) * (1 if pitch > 0 else -1)
        rear_pitch_displacement = rear_track_to_CG * np.tan(pitch) * (1 if pitch < 0 else -1)

        front_adjusted_heave = heave + front_pitch_displacement
        rear_adjusted_heave = heave + rear_pitch_displacement

        # TODO Implement these later

        adjusted_IAs = []
        for i in range(len(static_IA)):
            if i < 2:
                adjusted_IA = static_IA[i] + roll * roll_IA_gain[i] + front_adjusted_heave * heave_IA_gain[i]
            else:
                adjusted_IA = static_IA[i] + roll * roll_IA_gain[i] + rear_adjusted_heave * heave_IA_gain[i]

            adjusted_IAs.append(adjusted_IA)

        return adjusted_IAs

    # Support the above functions
    def _get_yaw_vel(self, lat_accel: float, vehicle_velocity: list[float]):
        # a_c = v^2 / r
        # omega = v / r -> r = v / omega
        # a_c = omega * v -> omega = a_c / v
        speed = np.sqrt(vehicle_velocity[0] ** 2 + vehicle_velocity[1] ** 2)
        yaw_vel = lat_accel / speed

        return yaw_vel

    def _get_IMF_vel(self, velocity: float, body_slip: float):
        IMF_velocity = velocity * np.array([np.cos(body_slip), np.sin(body_slip), 0])

        return IMF_velocity

    def _get_adj_steering(self, steering_angles: list[float], toe_angles: list[float]):
        adj_steering_angles = []
        for i in range(len(steering_angles)):
            adj_steering = steering_angles[i] + toe_angles[0]
            adj_steering_angles.append(adj_steering)

        return adj_steering_angles
