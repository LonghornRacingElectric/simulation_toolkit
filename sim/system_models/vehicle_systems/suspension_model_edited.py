from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vehicle_systems.tire_model import TireModel
from sim.util.math import coords

from scipy.optimize import fsolve
import numpy as np

# All coords with respect to SAE J670
class SuspensionModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        # Possibly add aero here
        self.tires = [TireModel(), TireModel(), TireModel(), TireModel()] # FL, FR, RL, RR

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
        front_FY_coeffs = vehicle_parameters.front_tire_coeff_Fy.get()
        front_FX_coeffs = vehicle_parameters.front_tire_coeff_Fx.get()
        rear_FY_coeffs = vehicle_parameters.rear_tire_coeff_Fy.get()
        rear_FX_coeffs = vehicle_parameters.rear_tire_coeff_Fx.get()

        coeff_array = [(front_FY_coeffs, front_FX_coeffs), (front_FY_coeffs, front_FX_coeffs), 
                       (rear_FY_coeffs, rear_FX_coeffs), (rear_FY_coeffs, rear_FX_coeffs)]

        # Heave, pitch, and roll
        vehicle_heave = state_vector.heave
        vehicle_pitch = state_vector.pitch
        vehicle_roll = state_vector.roll
        # Slip angle params
        steered_angles = self._get_steered_angles(vehicle_parameters, controls_vector.steering_angle)
        adj_steering_angles = self._get_adj_steering([steered_angles[0], steered_angles[1], 0, 0], vehicle_parameters.toe_angles.get())
        toe_angles = vehicle_parameters.toe_angles.get()
        yaw_rate = self._get_yaw_vel(state_vector.lateral_accel, self._get_IMF_vel(state_vector.velocity, state_vector.body_slip))
        vehicle_velocity = self._get_IMF_vel(state_vector.velocity, state_vector.body_slip)
        tire_positions = vehicle_parameters.tire_positions.get()
        # Inclination angle params
        static_IAs = vehicle_parameters.static_IAs.get()
        roll_IA_gain = vehicle_parameters.roll_IA_gain.get()
        heave_IA_gain = vehicle_parameters.heave_IA_gain.get()

        print(steered_angles)

        
        # Get FZ, SA, IA
        normal_loads = self._get_FZ_decoupled(vehicle_parameters = vehicle_parameters, heave = vehicle_heave, 
                                              pitch = vehicle_pitch, roll = vehicle_roll) 

        slip_angles = self._get_SA(vehicle_velocity = vehicle_velocity, adj_steering_angles = adj_steering_angles, 
                                   toe_angles = toe_angles, tire_position = tire_positions, yaw_rate = yaw_rate)
        
        inclination_angles = self._get_IA(vehicle_parameters = vehicle_parameters, adj_steering = adj_steering_angles, 
                                          heave = vehicle_heave, pitch = vehicle_pitch, roll = vehicle_roll, static_IA = static_IAs, 
                                          roll_IA_gain = roll_IA_gain, heave_IA_gain = heave_IA_gain)
        
        print(f'Normal loads: {normal_loads}')
        print(f'Slip angles: {slip_angles}')
        print(f'Inclination angles: {inclination_angles}')

        # Set fit coeffs
        for i in range(len(coeff_array)):
            self.tires[i].lat_coeffs = coeff_array[i][0]
            self.tires[i].long_coeffs = coeff_array[i][1]
        
        # Get tire output
        for i in range(len(self.tires)):
            tire_forces = self.tires[i]._get_comstock_forces(SR = 0, SA = slip_angles[i], FZ = normal_loads[i], IA = inclination_angles[i])
            observables_vector.tire_forces[i] = tire_forces
        
        pass
    
    # Top level functions
    # def _get_FZ_decoupled(self, vehicle_parameters: Car, heave: float, pitch: float, roll: float) -> list[float]:

    #     specific_residual_func = lambda x: self.__find_spring_displacements(x, heave, pitch, roll)
    #     tire_compression, wheel_displacement = fsolve(specific_residual_func, [0.006, 0.001])
    #     tire_compression = 0 if tire_compression < 0 else tire_compression
    #     normal_force = vehicle_parameters.front_tire_vertical_rate.get() * tire_compression
    #     normal_force = 0 if normal_force < 0 else normal_force

    #     return [FL, FR, RL, RR]

    # def __find_spring_displacements(self):
    #     guess_tire_compression, guess_wheel_displacement = x

    #     wheelrate = tire.wheelrate_f(guess_wheel_displacement) 
    #     tire_stiffness = tire.tire_stiffness_func(guess_tire_compression)
    #     riderate = (wheelrate * tire_stiffness) / (wheelrate + tire_stiffness) 


    #     ### ~~~ Roll Contribution ~~~ ###
    #     # TODO: do about roll center
    #     # TODO: for the tire & spring conversion to roll stiffness, assuming L & R have same stiffness here; seems like an issue
    #     tire_contribution = tire_stiffness * tire.trackwidth ** 2 / 2 * (1 if tire.is_left_tire else -1)
    #     spring_contribution = wheelrate * tire.trackwidth ** 2 / 2 * (1 if tire.is_left_tire else -1)
    #     arb_contribution = tire.arb_stiffness


    #     # ARB in parallel with spring, tire in series with ARB and spring
    #     roll_stiffness = - ((spring_contribution + arb_contribution) * tire_contribution /
    #                         ((spring_contribution + arb_contribution) + tire_contribution))
    #     f_roll = (roll_stiffness * roll) / tire.trackwidth
    #     x_wheel_roll = (spring_contribution / roll_stiffness) * f_roll / spring_contribution
        

    #     ### ~~~ Heave Contribution ~~~ ###
    #     f_heave = riderate * heave
    #     x_wheel_heave = f_heave / wheelrate


    #     ### ~~~ Pitch Contribution ~~~ ###
    #     # TODO: implement antisquat & antidive
    #     # TODO: do about pitch center
    #     pitch_heave = tire.position[0] * np.sin(pitch)
    #     f_pitch = riderate * pitch_heave
    #     x_wheel_pitch = f_pitch / wheelrate
        

    #     normal_force = f_roll + f_heave + f_pitch
    #     wheel_displacement = x_wheel_roll + x_wheel_heave + x_wheel_pitch
    #     tire_compression = normal_force / tire_stiffness

    #     self.logger.log(tire_name + "_tire_f_roll", f_roll)
    #     self.logger.log(tire_name + "_tire_f_heave", f_heave)
    #     self.logger.log(tire_name + "_tire_f_pitch", f_pitch)
    #     return guess_tire_compression - tire_compression, guess_wheel_displacement - wheel_displacement
    
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
    
    def _get_IA(self, vehicle_parameters: Car, adj_steering: list[float], heave: float, pitch: float, roll: float, static_IA: list[float],
                roll_IA_gain: list[float], heave_IA_gain: list[float]) -> float:
        
        front_track_to_CG = vehicle_parameters.wheelbase.get() * vehicle_parameters.cg_bias.get()
        rear_track_to_CG = vehicle_parameters.wheelbase.get() * (1 - vehicle_parameters.cg_bias.get())

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
        speed = np.sqrt(vehicle_velocity[0]**2 + vehicle_velocity[1]**2)
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