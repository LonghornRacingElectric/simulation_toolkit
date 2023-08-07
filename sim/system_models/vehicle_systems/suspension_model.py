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

        # I'm going to nest tires in suspension. Everything else I tried looks too messy

        tire_FL = TireModel()
        tire_FR = TireModel()
        tire_RL = TireModel()
        tire_RR = TireModel()

        self.controls_in = [
            "steering_angle"
        ]

        self.state_in = [
            "heave",
            "pitch",
            "roll",
            # "toe",
            "vehicle_velocity",
            # "lat_accel",
            # "tire_position_FL",
            # "tire_position_FR",
            # "tire_position_RL",
            # "tire_position_RR"
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
             state_dot_vector: StateDotVector, observables_out_vector: ObservablesVector):
        
        pass
    
    # Top level functions
    def _get_FZ_decoupled(self, heave: float, pitch: float, roll: float,
                          F_bump_springrate: float, F_roll_springrate: float, R_bump_springrate, R_roll_springrate: float,
                          F_bump_MR: float, F_roll_MR: float, R_bump_MR: float, R_roll_MR: float) -> list[float]:

        front_tire_stiffness = 1000
        rear_tire_stiffness = 1000

        front_track = 50 * 0.0254
        rear_track = 48 * 0.0254

        total_weight = 270 # kg
        cg_bias = 0.52

        # Heave contribution
        F_heave_rate_spring = F_bump_springrate / F_bump_MR**2
        F_heave_rate_tire = front_tire_stiffness * 2
        F_heave_rate = F_heave_rate_spring * F_heave_rate_tire / (F_heave_rate_spring + F_heave_rate_tire)  

        R_heave_rate_spring = R_bump_springrate / R_bump_MR**2
        R_heave_rate_tire = rear_tire_stiffness * 2
        R_heave_rate = R_heave_rate_spring * R_heave_rate_tire / (R_heave_rate_spring + R_heave_rate_tire)

        F_force_heave = F_heave_rate * heave
        R_force_heave = R_heave_rate * heave

        # Pitch contribution

        """
        Need to do this about pitch center. Calculate vertical displacements at the front and rear axle.
        Correlate to force using front and rear heave rates
        """

        # Roll contribution
        F_roll_rate_spring = 1/2 * front_track**2 * (F_roll_springrate / F_roll_MR**2)
        F_roll_rate_tire = 1/2 * front_track**2 * front_tire_stiffness
        F_roll_rate = F_roll_rate_spring * F_roll_rate_tire / (F_roll_rate_spring + F_roll_rate_tire)

        R_roll_rate_spring = 1/2 * rear_track**2 * (R_roll_springrate / R_roll_MR**2)
        R_roll_rate_tire = 1/2 * front_track**2 * rear_tire_stiffness
        R_roll_rate = R_roll_rate_spring * R_roll_rate_tire / (R_roll_rate_spring + R_roll_rate_tire)

        # roll_moment = tire_force * track_width -> tire_force = roll_moment / track_width
        F_roll_moment = F_roll_rate * roll
        R_roll_moment = R_roll_rate * roll

        F_roll_tire_force = F_roll_moment / front_track
        R_roll_tire_force = R_roll_moment = rear_track

        # Check on this. Do load transfer calc and use method above to solve for tire force due to roll. I believe tire force / 2
        # will match the load transfer. Plsss, this better be true :')
        
        # Load transfers
        FL_delta = -F_roll_tire_force / 2 + F_force_heave / 2
        FR_delta = F_roll_tire_force / 2 + F_force_heave / 2
        RL_delta = -R_roll_tire_force / 2 + R_force_heave / 2
        RR_delta = R_roll_tire_force / 2 + R_force_heave / 2

        # Static weights
        FL_static = total_weight * (1 - cg_bias) / 2
        FR_static = total_weight * (1 - cg_bias) / 2
        RL_static = total_weight * cg_bias / 2
        RR_static = total_weight * cg_bias / 2

        FL = FL_static + FL_delta
        FR = FR_static + FR_delta
        RL = RL_static + RL_delta
        RR = RR_static + RR_delta

        return [FL, FR, RL, RR]
    
    def _get_FZ_coupled(self, heave: float, pitch: float, roll: float) -> list[float]:
        return
    
    def _get_SA(self, vehicle_velocity: list[float], steering_angle, toe, tire_position: list[float], yaw_rate: float):

        tire_velocity_IMF = vehicle_velocity + np.cross(np.array([0, 0, yaw_rate]), tire_position)
        adj_steering = steering_angle + toe

        slip_angle = adj_steering - np.arctan2(tire_velocity_IMF[1], tire_velocity_IMF[0])

        return slip_angle
    
    def _get_IA(self, adj_steering: float, heave: float, pitch: float, roll: float, static_IA: float) -> float:

        # TODO Implement these later
        roll_IA_gain = 0
        heave_IA_gain = 0
        steer_IA_gain = 0
        pitch_IA_gain = 0

        adjusted_IA = static_IA + steer_IA_gain + roll_IA_gain + heave_IA_gain + pitch_IA_gain

        return adjusted_IA
    
    # Support the above functions
    def _yaw_vel(self, lat_accel: float, vehicle_velocity: list[float]):
        # a_c = v^2 / r
        # omega = v / r -> r = v / omega
        # a_c = omega * v -> omega = a_c / v
        speed = np.sqrt(vehicle_velocity[0]**2 + vehicle_velocity[1]**2)

        return lat_accel / speed