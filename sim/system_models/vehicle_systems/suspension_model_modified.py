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
from scipy.optimize import fsolve


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
        toe_angles = vehicle_parameters.toe_angles
        adjusted_steering_angles = [x + y for x, y in zip(steered_angles, toe_angles)]
        vehicle_velocity = self._get_IMF_vel(state_vector.velocity, state_vector.body_slip)
        yaw_rate = self._get_yaw_vel(state_vector.lateral_accel, vehicle_velocity)
        tire_positions = vehicle_parameters.tire_positions

        # Inclination angle params
        static_IAs = vehicle_parameters.static_IAs
        roll_IA_gain = vehicle_parameters.roll_IA_gain
        heave_IA_gain = vehicle_parameters.heave_IA_gain

        # Get FZ, SA, IA
        normal_loads = self._get_FZ_decoupled(vehicle_parameters=vehicle_parameters, heave=vehicle_heave,
                                              pitch=vehicle_pitch, roll=vehicle_roll)

        slip_angles = self._get_SA(vehicle_velocity=vehicle_velocity, steering_angles=steered_angles,
                                   toe_angles=toe_angles, tire_position=tire_positions, yaw_rate=yaw_rate)

        inclination_angles = self._get_IA(vehicle_parameters = vehicle_parameters, 
                                          adj_steering = steered_angles,
                                          toe_angles = toe_angles,
                                          heave = vehicle_heave, 
                                          pitch = vehicle_pitch, 
                                          roll = vehicle_roll, 
                                          static_IA = static_IAs,
                                          roll_IA_gain = roll_IA_gain, 
                                          heave_IA_gain = heave_IA_gain)
        
        observables_vector.normal_loads = normal_loads
        observables_vector.slip_angles = slip_angles
        observables_vector.inclination_angles = inclination_angles

        # Set fit coeffs
        for i in range(len(coeff_array)):
            self.tires[i].lat_coeffs = coeff_array[i][0]
            self.tires[i].long_coeffs = coeff_array[i][1]

        # Get tire output
        tire_outputs = []
        for i in range(len(self.tires)):
            tire_forces = self.tires[i]._get_comstock_forces(SR=0, SA=slip_angles[i], FZ=normal_loads[i],
                                                             IA=inclination_angles[i])
            
            tire_outputs.append([round(x) for x in tire_forces])
            vehicle_centric_forces = np.dot(coords.rotation_z(adjusted_steering_angles[i]), tire_forces) 
            vehicle_centric_moments = np.cross(vehicle_centric_forces, tire_positions[i])

            observables_vector.tire_forces_IMF[i] = vehicle_centric_forces
            observables_vector.tire_moments_IMF[i] = vehicle_centric_moments

        observables_vector.raw_tire_forces = tire_outputs

        self.aero.eval(vehicle_parameters, controls_vector, state_vector, state_dot_vector, observables_vector)

        aero_forces = np.array(observables_vector.aero_forces)
        aero_moments = np.array(observables_vector.aero_moments)

        # aero_forces = 0
        # aero_moments = 0

        sus_forces = np.array([x + y + z + w for x, y, z, w in zip(*observables_vector.tire_forces_IMF)])
        sus_moments = np.array([x + y + z + w for x, y, z, w in zip(*observables_vector.tire_moments_IMF)])

        gravity_forces = np.array([0, 0, -vehicle_parameters.total_mass * vehicle_parameters.accel_gravity])

        total_forces = aero_forces + sus_forces + gravity_forces
        total_moments = aero_moments + sus_moments

        observables_vector.forces_NTB = np.matmul(np.linalg.inv(coords.rotation_z(state_vector.body_slip)), total_forces)
        observables_vector.moments_NTB = np.matmul(np.linalg.inv(coords.rotation_z(state_vector.body_slip)), total_moments)

        # Initial force balance calculations
        yaw_acceleration = state_vector.yaw_accel
        translation_accelerations_imf = np.array([state_vector.long_accel, state_vector.lateral_accel, 0])
        translation_accelerations_ntb = np.matmul(np.linalg.inv(coords.rotation_z(state_vector.body_slip)), translation_accelerations_imf)

        # Sum of moments
        angular_accelerations = np.array([0, 0, yaw_acceleration])

        # Kinetic moments
        cg_relative_ntb = np.array([0, 0, vehicle_parameters.cg_height])
        m_a_term = np.cross(vehicle_parameters.total_mass * translation_accelerations_ntb, cg_relative_ntb)
        I_alpha_term = np.dot(vehicle_parameters.sprung_inertia, angular_accelerations)
        kinetic_moments = m_a_term + I_alpha_term
        summation_moments = kinetic_moments - observables_vector.moments_NTB

        inertial_forces = vehicle_parameters.total_mass * translation_accelerations_ntb
        summation_forces = inertial_forces - observables_vector.forces_NTB

        observables_vector.summation_forces = summation_forces
        observables_vector.summation_moments = summation_moments

        pass

    # Top level functions
    def _get_FZ_decoupled(self, vehicle_parameters: Car, heave: float, pitch: float, roll: float) -> list[float]:
        tire_FZ = []
        for i in range(len(self.tires)):
            specific_residual_func = lambda x: self._find_spring_displacements(x, vehicle_parameters, heave, pitch, roll, i)
            tire_compression, wheel_displacement = fsolve(specific_residual_func, [0.006, 0.001])
            tire_compression = 0 if tire_compression < 0 else tire_compression
            normal_force = vehicle_parameters.front_tire_vertical_rate * tire_compression
            normal_force = 0 if normal_force < 0 else normal_force

            tire_FZ.append(normal_force)

        return tire_FZ
    
    def _find_spring_displacements(self, x, params: Car, heave: float, pitch: float, roll:float, tire_index: int):
        heave_spring_rates = [params.front_heave_springrate, params.front_heave_springrate, params.rear_heave_springrate, params.rear_heave_springrate]
        heave_MRs = [params.front_heave_MR, params.front_heave_MR, params.rear_heave_MR, params.rear_heave_MR]
        
        roll_spring_rates = [params.front_roll_springrate, params.front_roll_springrate, params.rear_roll_springrate, params.rear_roll_springrate]
        roll_MRs = [params.front_roll_MR, params.front_roll_MR, params.rear_roll_MR, params.rear_roll_MR]

        tire_rates = [params.front_tire_vertical_rate, params.front_tire_vertical_rate, params.rear_tire_vertical_rate, params.rear_tire_vertical_rate]

        track_widths = [params.front_track, params.front_track, params.rear_track, params.rear_track]

        guess_tire_compression, guess_wheel_displacement = x

        # Rates
        heave_wheelrate = heave_spring_rates[tire_index] / (heave_MRs[tire_index]**2)
        roll_wheelrate = roll_spring_rates[tire_index] / (roll_MRs[tire_index]**2)
        tire_stiffness = tire_rates[tire_index]
        riderate = (heave_wheelrate * tire_stiffness) / (heave_wheelrate + tire_stiffness) 


        ### ~~~ Roll Contribution ~~~ ###
        # TODO: do about roll center
        # TODO: for the tire & spring conversion to roll stiffness, assuming L & R have same stiffness here; seems like an issue
        tire_contribution = tire_stiffness * track_widths[tire_index] ** 2 / 2
        spring_contribution = roll_wheelrate * track_widths[tire_index] ** 2 / 2


        # ARB in parallel with spring, tire in series with ARB and spring
        roll_stiffness = ((spring_contribution) * tire_contribution / ((spring_contribution) + tire_contribution))
        f_roll = (roll_stiffness * roll) / track_widths[tire_index]
        x_wheel_roll = (spring_contribution / roll_stiffness) * f_roll / spring_contribution
        

        ### ~~~ Heave Contribution ~~~ ###
        f_heave = riderate * heave
        x_wheel_heave = f_heave / heave_wheelrate


        ### ~~~ Pitch Contribution ~~~ ###
        # TODO: implement antisquat & antidive
        # TODO: do about pitch center
        pitch_heave = params.tire_positions[tire_index][0] * np.sin(pitch)
        f_pitch = riderate * pitch_heave
        x_wheel_pitch = f_pitch / heave_wheelrate
        

        normal_force = f_roll + f_heave + f_pitch
        wheel_displacement = x_wheel_roll + x_wheel_heave + x_wheel_pitch
        tire_compression = normal_force / tire_stiffness

        return guess_tire_compression - tire_compression, guess_wheel_displacement - wheel_displacement

    def _get_FZ_coupled(self, heave: float, pitch: float, roll: float) -> list[float]:
        return

    def _get_steered_angles(self, vehicle_parameters: Car, steered_angle: float):
        outer_angle = 0.28166 * steered_angle - 0.00983
        inner_angle = 0.24888 * steered_angle + 0.0010

        if steered_angle == 0:
            return [0, 0]
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
            slip_angle = (steering_angles[i] + toe_angles[i]) + np.arctan2(tire_IMF_velocities[i][1], tire_IMF_velocities[i][0])
            slip_angles.append(slip_angle)

        return slip_angles

    def _get_IA(self, vehicle_parameters: Car, adj_steering: list[float], toe_angles: list[float], 
                heave: float, pitch: float, roll: float, static_IA: list[float], roll_IA_gain: list[float], 
                heave_IA_gain: list[float]) -> list[float]:

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
        if lat_accel == 0:
            yaw_vel = 0
        else:
            speed = np.sqrt(vehicle_velocity[0] ** 2 + vehicle_velocity[1] ** 2)
            yaw_vel = lat_accel / speed

        return yaw_vel

    def _get_IMF_vel(self, velocity: float, body_slip: float):
        IMF_velocity = velocity * np.array([np.cos(body_slip), np.sin(body_slip), 0])

        return IMF_velocity