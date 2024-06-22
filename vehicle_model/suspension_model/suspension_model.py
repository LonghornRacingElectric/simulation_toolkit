import numpy
import numpy as np
from numpy import ndarray

# All coords with respect to SAE J670
class SuspensionModel(VehicleSystemModel):
    def __init__(self, transient=False):
        super().__init__()

        self.transient = transient

        # Initialize tire model
        self.tires = [TireModel(), TireModel(), TireModel(), TireModel()]  # [FL, FR, RL, RR]

        self.controls_in = [
            "steering_angle"
        ]

        self.state_in = [
            "heave",
            "heave_dot",
            "heave_dot_dot",
            
            "pitch",
            "pitch_dot",
            "pitch_dot_dot",
            
            "roll",
            "roll_dot",
            "roll_dot_dot"

            "yaw_dot",
            "yaw_dot_dot",

            "long_vel",
            "long_accel",

            "lat_vel",
            "lat_accel",
        ]

        self.state_out = [
            # TODO: Transient or nah (I wanna say yaw)
        ]

        self.observables_out = [
            "FL_RH",
            "FR_RH",
            "RL_RH",
            "RR, RH",

            "Fr_crank",
            "Rr_crank",
            
            "Fr_spring",
            "Rr_spring"
        ]

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_vector: StateVector,
             state_dot_vector: StateDotVector, observables_vector: ObservablesVector) -> None:

        # Initialize heave, pitch, roll, long accel, lat accel, yaw accel, steered angle, body slip, velocity
        chassis_heave = state_vector.heave
        chassis_pitch = state_vector.pitch
        chassis_roll = state_vector.roll
        steered_angle = controls_vector.steering_angle
        body_slip = state_vector.body_slip
        velocity = state_vector.speed

        # Initialize additional slip angle params
        toe_angles = vehicle_parameters.toe_angles
        tire_positions = vehicle_parameters.tire_positions

        # Secondary slip angle calculations
        steered_angles = self._get_steered_angles(vehicle_parameters=vehicle_parameters,
                                                  steered_angle=steered_angle)
        adjusted_steering_angles = np.array(steered_angles) + np.array(toe_angles)
        IMF_velocity = self._get_IMF_vel(velocity=velocity,
                                         body_slip=body_slip)

        if self.transient:
            yaw_rate = state_vector.yaw_rate
        else:
            translational_accelerations_IMF = np.array([observables_vector.long_accel, observables_vector.lateral_accel, 0])
            translational_accelerations_NTB = np.matmul(coords.rotation_z(body_slip), translational_accelerations_IMF)
            yaw_rate = self._get_yaw_vel(lateral_accel=float(translational_accelerations_NTB[1]),
                                         vehicle_velocity=IMF_velocity)

        # Initialize inclination angle params
        static_IAs = vehicle_parameters.static_IAs

        FL_IA_gain = vehicle_parameters.FL_IA_gain
        FR_IA_gain = vehicle_parameters.FR_IA_gain
        RL_IA_gain = vehicle_parameters.RL_IA_gain
        RR_IA_gain = vehicle_parameters.RR_IA_gain

        IA_response_surfaces = [FL_IA_gain, FR_IA_gain, RL_IA_gain, RR_IA_gain]

        # Calculate normal loads, slip angles, and inclination angles for each tire [Fl, FR, RL, RR]
        normal_loads = self._get_FZ_decoupled(vehicle_parameters=vehicle_parameters,
                                              heave=chassis_heave,
                                              pitch=chassis_pitch,
                                              roll=chassis_roll)

        slip_angles, tire_IMF_velocities = self._get_SA(vehicle_velocity=IMF_velocity,
                                                        steering_angles=steered_angles,
                                                        toe_angles=toe_angles,
                                                        tire_position=tire_positions,
                                                        yaw_rate=yaw_rate)

        slip_ratios = state_vector.wheel_slip_ratios

        inclination_angles = self._get_IA(vehicle_parameters=vehicle_parameters,
                                          IA_response=IA_response_surfaces,
                                          adj_steering=steered_angles,
                                          heave=chassis_heave,
                                          pitch=chassis_pitch,
                                          roll=chassis_roll,
                                          static_IA=static_IAs)

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

        # Get tire output
        tire_outputs = []
        tire_torques = [0, 0, 0, 0]
        for i in range(len(self.tires)):
            # Calculate tire rotational velocities
            tire_heading_vector = np.array(
                [np.cos(adjusted_steering_angles[i]), np.sin(adjusted_steering_angles[i]), 0])
            tire_heading_unit_vector = tire_heading_vector / np.linalg.norm(tire_heading_vector)
            tire_heading_velocity = np.dot(tire_IMF_velocities[i], tire_heading_unit_vector)
            # wheel_speed = (slip_ratios[i] + 1) * tire_heading_velocity / vehicle_parameters.tire_radii[i]

            max_force = None

            if self.transient:
                if state_vector.wheel_angular_velocities[i] == 0:
                    max_force = state_dot_vector.powertrain_torques[i] / vehicle_parameters.tire_radii[i]
                if tire_heading_velocity < 0.1:
                    slip_angles[i] = 0

            tire_forces = self.tires[i].get_comstock_forces(SR=slip_ratios[i],
                                                            SA=slip_angles[i],
                                                            FZ=normal_loads[i],
                                                            IA=inclination_angles[i],
                                                            max_force=max_force)
            try:
                tire_outputs.append([round(x) for x in tire_forces])
            except:
                break

            # Rotate tire forces to correspond with true steered angle (now in terms of force on the vehicle)
            vehicle_centric_forces = np.matmul(coords.rotation_z(adjusted_steering_angles[i]), tire_forces)

            # Calculate vehicle moments due to tire forces
            vehicle_centric_moments = np.cross(tire_positions[i], vehicle_centric_forces)

            # Calculate tire torques
            tire_torque = vehicle_centric_forces[0] * vehicle_parameters.tire_radii[i]

            tire_torques[i] = tire_torque

            # Log force and moment calculations from above
            observables_vector.tire_heading_velocities[i] = tire_heading_velocity
            observables_vector.tire_forces_IMF[i] = vehicle_centric_forces
            observables_vector.tire_moments_IMF[i] = vehicle_centric_moments
            # observables_vector.wheel_angular_velocities[i] = wheel_speed

        # Log additional desired parameters
        observables_vector.average_steered_angle = (steered_angles[0] + steered_angles[1]) / 2
        observables_vector.normal_loads = normal_loads
        observables_vector.slip_angles = slip_angles
        observables_vector.inclination_angles = inclination_angles
        observables_vector.tire_model_force_outputs = tire_outputs

        # sum forces & moments
        state_dot_vector.sus_forces = np.array([x + y + z + w for x, y, z, w in zip(*observables_vector.tire_forces_IMF)])
        state_dot_vector.sus_moments = np.array([x + y + z + w for x, y, z, w in zip(*observables_vector.tire_moments_IMF)])
        state_dot_vector.tire_torques = tire_torques

        # print()
        # for i in range(4):
        #     print(f"tire {i} F", observables_vector.tire_forces_IMF[i])
        #     print(f"tire {i} M", observables_vector.tire_moments_IMF[i])
        # print()
        # print("SR", slip_ratios)
        # print("SA", slip_angles)
        # print("FX", state_dot_vector.sus_forces[0])
        # print("FY", state_dot_vector.sus_forces[1])
        # print()

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

        F_force_heave = (F_heave_rate * F_adjusted_heave) / (1 - vehicle_parameters.front_anti)
        R_force_heave = (R_heave_rate * R_adjusted_heave) / (1 - vehicle_parameters.rear_anti)

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
        return []

    def _get_steered_angles(self, vehicle_parameters: Car, steered_angle: float) -> list[float]:
        inner_angle_response = vehicle_parameters.FI_steering_response
        outer_angle_response = vehicle_parameters.FO_steering_response

        inner_angle = inner_angle_response(abs(steered_angle))
        outer_angle = outer_angle_response(abs(steered_angle))

        if steered_angle > 0:
            return [inner_angle, outer_angle, 0, 0]
        else:
            return [-outer_angle, -inner_angle, 0, 0]

    def _get_SA(self, vehicle_velocity: ndarray[float], steering_angles: list[float], toe_angles: list[float],
                tire_position: list[float], yaw_rate: float) -> list[list[float]]:

        tire_IMF_velocities = []
        for i in range(len(tire_position)):
            tire_velocity_IMF = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]), np.array(tire_position[i]))
            tire_IMF_velocities.append(tire_velocity_IMF)

        slip_angles = []
        for i in range(len(steering_angles)):
            slip_angle = (steering_angles[i] + toe_angles[i]) - np.arctan2(tire_IMF_velocities[i][1],
                                                                           tire_IMF_velocities[i][0])
            slip_angles.append(slip_angle)

        return [slip_angles, tire_IMF_velocities]

    def _get_IA(self, vehicle_parameters: Car, IA_response: list[Callable], adj_steering: list[float], heave: float,
                pitch: float, roll: float, static_IA: list[float]) -> list[float]:

        front_track_to_CG = vehicle_parameters.wheelbase * vehicle_parameters.cg_bias
        rear_track_to_CG = vehicle_parameters.wheelbase * (1 - vehicle_parameters.cg_bias)

        front_pitch_displacement = front_track_to_CG * np.tan(abs(pitch))
        rear_pitch_displacement = rear_track_to_CG * np.tan(abs(pitch))

        F_heave = heave + front_pitch_displacement
        R_heave = heave + rear_pitch_displacement

        heave_lst = [F_heave, F_heave, R_heave, R_heave]

        adjusted_IA = []
        for i in range(len(static_IA)):
            IA_gain = IA_response[i](heave_lst[i], roll)
            adjusted_IA.append(static_IA[i] + IA_gain)

        return adjusted_IA

    def _get_yaw_vel(self, lateral_accel: float, vehicle_velocity: ndarray[float]) -> float:
        # a_c = v^2 / r
        # omega = v / r -> r = v / omega
        # a_c = omega * v -> omega = a_c / v
        # omega = a_c / v
        if lateral_accel == 0:
            yaw_vel = 0

        else:
            yaw_vel = lateral_accel / np.linalg.norm(vehicle_velocity)

        return yaw_vel

    def _get_IMF_vel(self, velocity: float, body_slip: float) -> ndarray[float]:
        IMF_velocity = velocity * np.array([np.cos(body_slip), np.sin(body_slip), 0])
        return IMF_velocity
