from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vehicle_systems.tire_model import TireModel

import math
import numpy as np

# All coords with respect to SAE J670
class SuspensionModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        # Possibly add aero here
        self.tires = [TireModel(), TireModel(), TireModel(), TireModel()] # FL, FR, RL, RR

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
            "vehicle_FX",
            "vehicle_FY",
            "vehicle_FZ",
            "vehicle_MX",
            "vehicle_MY",
            "vehicle_MZ"
        ]

        # Intermediate steps (or extra information)
        self.observables_out = [

        ]

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_vector: StateVector,
             state_dot_vector: StateDotVector, observables_vector: ObservablesVector):
        
        normal_loads = self._get_FZ_decoupled(vehicle_parameters = 
                                    vehicle_parameters, 
                                    heave = 
                                    state_vector.heave, 
                                    pitch = 
                                    state_vector.pitch, 
                                    roll = 
                                    state_vector.roll)

        slip_angles = self._get_SA(vehicle_velocity = 
                          self._get_IMF_vel(state_vector.velocity, state_vector.body_slip), 
                          adj_steering_angles = 
                          self._get_adj_steering([state_vector.steered_angle, state_vector.steered_angle, 0, 0], vehicle_parameters.toe_angles.get()),
                          yaw_rate = 
                          self._get_yaw_vel(state_vector.lateral_accel, self._get_IMF_vel(state_vector.velocity, state_vector.body_slip)))
        
        inclination_angles = self._get_IA(adj_steering_angles = 
                          self._get_adj_steering([state_vector.steered_angle, state_vector.steered_angle, 0, 0], vehicle_parameters.toe_angles.get()),
                          heave = 
                          state_vector.heave,
                          pitch = 
                          state_vector.pitch,
                          roll =
                          state_vector.roll,
                          static_IA = 
                          vehicle_parameters.static_IAs.get(),
                          roll_IA_gain = 
                          vehicle_parameters.roll_IA_gain.get(),
                          heave_IA_gain = 
                          vehicle_parameters.heave_IA_gain.get())
        
        tire_output = []
        for i in range(self.tires):
            tire_forces = self.tires[i]._get_comstock_forces(SR = 0,
                                               SA = slip_angles[i],
                                               FZ = normal_loads[i],
                                               IA = inclination_angles[i])
            tire_output.append(tire_forces)
        
        pass
    
    # Top level functions
    def _get_FZ_decoupled(self, vehicle_parameters: Car, heave: float, pitch: float, roll: float) -> list[float]:

        # Heave contribution
        F_heave_rate_spring = vehicle_parameters.front_bump_springrate.get() / vehicle_parameters.front_bump_MR.get()**2
        F_heave_rate_tire = vehicle_parameters.front_tire_vertical_rate.get() * 2
        F_heave_rate = F_heave_rate_spring * F_heave_rate_tire / (F_heave_rate_spring + F_heave_rate_tire)  

        R_heave_rate_spring = vehicle_parameters.rear_bump_springrate.get() / vehicle_parameters.rear_bump_MR.get()**2
        R_heave_rate_tire = vehicle_parameters.rear_tire_vertical_rate.get() * 2
        R_heave_rate = R_heave_rate_spring * R_heave_rate_tire / (R_heave_rate_spring + R_heave_rate_tire)

        F_force_heave = F_heave_rate * heave
        R_force_heave = R_heave_rate * heave

        # Pitch contribution

        """
        Need to do this about pitch center. Calculate vertical displacements at the front and rear axle.
        Correlate to force using front and rear heave rates
        """

        # Roll contribution
        F_roll_rate_spring = 1/2 * vehicle_parameters.front_track.get()**2 * (vehicle_parameters.front_roll_springrate.get() / vehicle_parameters.front_roll_MR.get()**2)
        F_roll_rate_tire = 1/2 * vehicle_parameters.front_track.get()**2 * vehicle_parameters..get() front_tire_vertical_rate
        F_roll_rate = F_roll_rate_spring * F_roll_rate_tire / (F_roll_rate_spring + F_roll_rate_tire)

        R_roll_rate_spring = 1/2 * vehicle_parameters.rear_track.get()**2 * (vehicle_parameters.rear_roll_springrate.get() / vehicle_parameters.rear_roll_MR.get()**2)
        R_roll_rate_tire = 1/2 * vehicle_parameters.rear_track.get()**2 * vehicle_parameters.rear_tire_vertical_rate.get()
        R_roll_rate = R_roll_rate_spring * R_roll_rate_tire / (R_roll_rate_spring + R_roll_rate_tire)

        # roll_moment = tire_force * track_width -> tire_force = roll_moment / track_width
        F_roll_moment = F_roll_rate * roll
        R_roll_moment = R_roll_rate * roll

        F_roll_tire_force = F_roll_moment / vehicle_parameters.front_track.get()
        R_roll_tire_force = R_roll_moment / vehicle_parameters.rear_track.get()

        # Check on this. Do load transfer calc and use method above to solve for tire force due to roll. I believe tire force / 2
        # will match the load transfer. Plsss, this better be true :')
        
        # Load transfers
        FL_delta = -F_roll_tire_force / 2 + F_force_heave / 2
        FR_delta = F_roll_tire_force / 2 + F_force_heave / 2
        RL_delta = -R_roll_tire_force / 2 + R_force_heave / 2
        RR_delta = R_roll_tire_force / 2 + R_force_heave / 2

        # Static weights
        FL_static = vehicle_parameters.total_mass.get() * vehicle_parameters.accel_gravity.get() * (1 - vehicle_parameters.cg_bias.get()) / 2
        FR_static = vehicle_parameters.total_mass.get() * vehicle_parameters.accel_gravity.get() * (1 - vehicle_parameters.cg_bias.get()) / 2
        RL_static = vehicle_parameters.total_mass.get() * vehicle_parameters.accel_gravity.get() * vehicle_parameters.cg_bias.get() / 2
        RR_static = vehicle_parameters.total_mass.get() * vehicle_parameters.accel_gravity.get() * vehicle_parameters.cg_bias.get() / 2

        FL = FL_static + FL_delta
        FR = FR_static + FR_delta
        RL = RL_static + RL_delta
        RR = RR_static + RR_delta

        return [FL, FR, RL, RR]
    
    def _get_FZ_coupled(self, heave: float, pitch: float, roll: float) -> list[float]:
        return
    
    def _get_SA(self, vehicle_velocity: list[float], adj_steering_angles: list[float], toe_angles: list[float], 
                tire_position: list[float], yaw_rate: float):

        tire_IMF_velocities = []
        for i in range(len(tire_position)):
            tire_velocity_IMF = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]), tire_position[i])
            tire_IMF_velocities.append(tire_velocity_IMF)

        slip_angles = []
        for i in range(len(adj_steering_angles)):
            slip_angle = adj_steering_angles[i] - np.arctan2(tire_velocity_IMF[i][1], tire_velocity_IMF[i][0])
            slip_angles.append(slip_angle)

        return slip_angles
    
    def _get_IA(self, adj_steering: list[float], heave: float, pitch: float, roll: float, static_IA: list[float],
                roll_IA_gain: list[float], heave_IA_gain: list[float]) -> float:
        
        # Convert pitch into adjusted heave
        # Just heave and roll after that

        front_adjusted_heave = heave + 0
        rear_adjusted_heave = heave + 0

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

        return lat_accel / speed
    
    def _get_IMF_vel(self, velocity: float, body_slip: float):
         IMF_velocity = velocity * np.array([np.cos(body_slip), np.sin(body_slip), 0])

         return velocity
    
    def _get_adj_steering(self, steering_angles: list[float], toe_angles: list[float]):
        adj_steering_angles = []
        for i in range(steering_angles):
            adj_steering = steering_angles[i] + toe_angles[0]
            adj_steering_angles.append(adj_steering)
        
        return adj_steering_angles