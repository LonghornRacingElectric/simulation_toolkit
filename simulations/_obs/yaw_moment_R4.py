from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_model import SuspensionModel
from vehicle_model._assets.misc_linalg import rotation_matrix
from vehicle_model._assets.misc_linalg import plane_eval
from vehicle_model.vehicle_model import VehicleModel
from LHR_tire_toolkit.MF52 import MF52
from typing import Sequence, Tuple
from scipy.optimize import minimize
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import numpy as np
import time


class YMD:
    def __init__(self, vehicle: VehicleModel, mesh: float) -> None:
        self.vehicle = vehicle
        self.mesh = mesh

        self.delta_sweep = np.linspace(-30 * 3.50 / 360 * 0.0254, 30 * 3.50 / 360 * 0.0254, mesh)
        self.beta_sweep = np.linspace(-8 * np.pi / 180, 8 * np.pi / 180, mesh)
    
    def generate_constant_velocity_YMD(self, velocity: float) -> None:
        self.body_slip_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.steered_angle_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.all_points = []

        total_states = len(self.delta_sweep) * len(self.beta_sweep)
        counter = 0
        total_start = time.time()

        y_ddot_lst = []
        yaw_ddot_lst = []
        delta_lst = []
        Ay_lst = []

        for i, beta in enumerate(self.beta_sweep):
            for j, delta in enumerate(self.delta_sweep):
                print(f"Progress | {round(counter / total_states * 100, 1)}%", end="\r")
                counter += 1
                x_ddot, y_ddot, yaw_ddot, heave, pitch, roll = minimize(self._residual_function, \
                                        x0=[0, 0, 0, 0, 0, 0], args=[delta, beta, velocity], method="SLSQP").x

                self.steered_angle_iso_lines[j][0] = delta / (3.50 / 360 * 0.0254)
                self.steered_angle_iso_lines[i][1][j] = y_ddot
                self.steered_angle_iso_lines[i][2][j] = yaw_ddot \
                # / (self.vehicle.total_mass * 9.81 * abs(self.vehicle.suspension.FL_double_wishbone.contact_patch.position[0] - self.vehicle.suspension.RL_double_wishbone.contact_patch.position[0]))
                self.body_slip_iso_lines[i][0] = beta
                self.body_slip_iso_lines[j][1][i] = y_ddot
                self.body_slip_iso_lines[j][2][i] = yaw_ddot \
                #  / (self.vehicle.total_mass * 9.81 * abs(self.vehicle.suspension.FL_double_wishbone.contact_patch.position[0] - self.vehicle.suspension.RL_double_wishbone.contact_patch.position[0]))

                y_ddot_lst.append(y_ddot)
                yaw_ddot_lst.append(yaw_ddot)
                if beta == 0:
                    delta_lst.append(delta / (3.50 / 360 * 0.0254))
                    Ay_lst.append(y_ddot / 9.81)

        # plt.plot(Ay_lst, delta_lst)
        # plt.show()
        self.plot()

    def _residual_function(self, x: Sequence[float], args: Sequence[float]) -> Sequence[float]:
        """
        ## Residual Function

        Calculates state convergence
        - This is where all the calculations happen, so check this out
        - IMF aligned with vehicle reference frame

        Parameters
        ----------
        x : Sequence[float]
            Solution guess
            - [x_ddot, y_ddot, yaw_ddot, heave, pitch, roll]
        args : Sequence[float]
            - [delta, beta, velocity]

        Returns
        -------
        Sequence[float]
            Residuals
        """

        ax = x[0]
        ay = x[1]
        yaw_ddot = x[2]
        heave = x[3]
        pitch = x[4]
        roll = x[5]

        delta = args[0]
        beta = args[1]
        velocity = args[2]

        #############
        ### Setup ###
        #############

        # Adjust velocity vectors
        imf_velocity = velocity * np.array([np.cos(beta), np.sin(beta), 0])

        # Adjust acceleration vectors and dependencies
        imf_accel = np.array([ax, ay, 0])
        ntb_accel = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=beta), imf_accel)
        yaw_rate = 0 if ay == 0 else ntb_accel[1] / np.linalg.norm(imf_velocity)

        # Store suspension objects
        suspension = self.vehicle.suspension

        # Store tire objects
        FL_tire = self.vehicle.FL_tire
        FR_tire = self.vehicle.FR_tire
        RL_tire = self.vehicle.RL_tire
        RR_tire = self.vehicle.RR_tire

        # Store suspension corner objects
        FL_weight = suspension.Fr_axle.left.weight
        FR_weight = suspension.Fr_axle.right.weight
        RL_weight = suspension.Rr_axle.left.weight
        RR_weight = suspension.Rr_axle.right.weight

        # Store cg position
        cgx = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgy = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgz = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.cg_pos = np.array([cgx, cgy, cgz])

        # Store contact patch positions
        FL_dw_cpx = suspension.FL_cpx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FL_dw_cpy = suspension.FL_cpy_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FL_dw_cpz = suspension.FL_cpz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_dw_cpx = suspension.FR_cpx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_dw_cpy = suspension.FR_cpy_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_dw_cpz = suspension.FR_cpz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_dw_cpx = suspension.RL_cpx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_dw_cpy = suspension.RL_cpy_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_dw_cpz = suspension.RL_cpz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_dw_cpx = suspension.RR_cpx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_dw_cpy = suspension.RR_cpy_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_dw_cpz = suspension.RR_cpz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FL_dw_cp_pos = np.array([FL_dw_cpx, FL_dw_cpy, FL_dw_cpz])
        FR_dw_cp_pos = np.array([FR_dw_cpx, FR_dw_cpy, FR_dw_cpz])
        RL_dw_cp_pos = np.array([RL_dw_cpx, RL_dw_cpy, RL_dw_cpz])
        RR_dw_cp_pos = np.array([RR_dw_cpx, RR_dw_cpy, RR_dw_cpz])

        # Store FAP positions
        FL_FV_FAPz = suspension.FL_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_FV_FAPz = suspension.FR_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_FV_FAPz = suspension.RL_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_FV_FAPz = suspension.RR_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        FL_SV_FAPz = suspension.FL_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_SV_FAPz = suspension.FR_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_SV_FAPz = suspension.RL_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_SV_FAPz = suspension.RR_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        
        # Store roll stiffnesses
        Fr_Kr = suspension.Fr_Kr_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        Rr_Kr = suspension.Rr_Kr_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # Store pitch stiffnesses
        Kp = suspension.Kp_lookup

        # Tire shit
        FL_gamma = suspension.FL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_gamma = suspension.FR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_gamma = suspension.RL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_gamma = suspension.RR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        FL_toe = suspension.FL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_toe = suspension.FR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_toe = suspension.RL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_toe = suspension.RR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        Fr_track = abs(FL_dw_cp_pos[1] - FR_dw_cp_pos[1])
        Rr_track = abs(RL_dw_cp_pos[1] - RR_dw_cp_pos[1])

        left_wheelbase = abs(FL_dw_cp_pos[0] - RL_dw_cp_pos[0])
        right_wheelbase = abs(FR_dw_cp_pos[0] - RR_dw_cp_pos[0])

        self.FL_Cp_wrt_cg = FL_dw_cp_pos - self.cg_pos
        self.FR_Cp_wrt_cg = FR_dw_cp_pos - self.cg_pos
        self.RL_Cp_wrt_cg = RL_dw_cp_pos - self.cg_pos
        self.RR_Cp_wrt_cg = RR_dw_cp_pos - self.cg_pos

        # Calculate normal loads
        
        # Inelastic lateral load transfer
        FL_inelastic_lat = FL_weight / self.vehicle.environment["G"] * FL_FV_FAPz * ay / Fr_track
        FR_inelastic_lat = FR_weight / self.vehicle.environment["G"] * FR_FV_FAPz * ay / Fr_track
        RL_inelastic_lat = RL_weight / self.vehicle.environment["G"] * RL_FV_FAPz * ay / Rr_track
        RR_inelastic_lat = RR_weight / self.vehicle.environment["G"] * RR_FV_FAPz * ay / Rr_track
        
        Fr_inelastic_lat = FL_inelastic_lat + FR_inelastic_lat
        Rr_inelastic_lat = RL_inelastic_lat + RR_inelastic_lat
        
        # Inelastic longitudinal load transfer
        FL_inelastic_long = FL_weight / self.vehicle.environment["G"] * FL_SV_FAPz * ax / left_wheelbase
        FR_inelastic_long = FR_weight / self.vehicle.environment["G"] * FR_SV_FAPz * ax / right_wheelbase
        RL_inelastic_long = RL_weight / self.vehicle.environment["G"] * RL_SV_FAPz * ax / left_wheelbase
        RR_inelastic_long = RR_weight / self.vehicle.environment["G"] * RR_SV_FAPz * ax / right_wheelbase
        
        left_inelastic_long = FL_inelastic_long + RL_inelastic_long
        right_inelastic_long = FR_inelastic_long + RR_inelastic_long
        
        # Elastic lateral load transfer
        FL_elastic_lat_inertial = FL_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FL_FV_FAPz) * ay
        FR_elastic_lat_inertial = FR_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FR_FV_FAPz) * ay
        RL_elastic_lat_inertial = RL_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RL_FV_FAPz) * ay
        RR_elastic_lat_inertial = RR_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RR_FV_FAPz) * ay
        
        Fr_elastic_lat = (FL_elastic_lat_inertial + FR_elastic_lat_inertial) / suspension.Fr_axle.track_width
        Rr_elastic_lat = (RL_elastic_lat_inertial + RR_elastic_lat_inertial) / suspension.Rr_axle.track_width

        total_elastic_lt_lat = Fr_elastic_lat + Rr_elastic_lat

        # Elastic longitudinal load transfer
        FL_elastic_long_inertial = FL_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FL_SV_FAPz) * ax
        FR_elastic_long_inertial = FR_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FR_SV_FAPz) * ax
        RL_elastic_long_inertial = RL_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RL_SV_FAPz) * ax
        RR_elastic_long_inertial = RR_weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RR_SV_FAPz) * ax
        
        left_elastic_long = (FL_elastic_long_inertial + RL_elastic_long_inertial) / left_wheelbase
        right_elastic_long = (FR_elastic_long_inertial + RR_elastic_long_inertial) / right_wheelbase

        # Total load transfers
        Fr_lat_lt = Fr_inelastic_lat + Fr_Kr / (Fr_Kr + Rr_Kr) * total_elastic_lt_lat
        Rr_lat_lt = Rr_inelastic_lat + Rr_Kr / (Fr_Kr + Rr_Kr) * total_elastic_lt_lat

        left_long_lt = left_inelastic_long + left_elastic_long
        right_long_lt = right_inelastic_long + right_elastic_long
        
        FL_Fz = FL_weight - Fr_lat_lt - left_long_lt
        FR_Fz = FR_weight + Fr_lat_lt - right_long_lt
        RL_Fz = RL_weight - Rr_lat_lt + left_long_lt
        RR_Fz = RR_weight + Rr_lat_lt + right_long_lt
        calculated_Fz = np.array([FL_Fz, FR_Fz, RL_Fz, RR_Fz])

        # Steady state load transfers
        Fr_lat_LT = (FL_weight + FR_weight) * ay / self.vehicle.environment["G"] * self.cg_pos[2] / Fr_track
        Rr_lat_LT = (RL_weight + RR_weight) * ay / self.vehicle.environment["G"] * self.cg_pos[2] / Rr_track
        left_long_LT = self.cg_pos[2] / left_wheelbase * (FL_weight + RL_weight) * ax / self.vehicle.environment["G"]
        right_long_LT = self.cg_pos[2] / right_wheelbase * (FR_weight + RR_weight) * ax / self.vehicle.environment["G"]
        
        FL_Fz_LT = FL_weight - Fr_lat_LT - left_long_LT
        FR_Fz_LT = FR_weight + Fr_lat_LT - right_long_LT
        RL_Fz_LT = RL_weight - Rr_lat_LT + left_long_LT
        RR_Fz_LT = RR_weight + Rr_lat_LT + right_long_LT
        ss_Fz = np.array([FL_Fz_LT, FR_Fz_LT, RL_Fz_LT, RR_Fz_LT])

        Fz_resid = calculated_Fz - ss_Fz

        # Wheel velocities
        FL_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.FL_Cp_wrt_cg)
        FR_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.FR_Cp_wrt_cg)
        RL_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.RL_Cp_wrt_cg)
        RR_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.RR_Cp_wrt_cg)

        # Slip angles
        FL_alpha = FL_toe - np.arctan2(FL_velocity[1], FL_velocity[0])
        FR_alpha = FR_toe - np.arctan2(FR_velocity[1], FR_velocity[0])
        RL_alpha = RL_toe - np.arctan2(RL_velocity[1], RL_velocity[0])
        RR_alpha = RR_toe - np.arctan2(RR_velocity[1], RR_velocity[0])

        # Tire loads
        self.FL_loads = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[0:3]
        self.FR_loads = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[0:3]
        self.RL_loads = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[0:3]
        self.RR_loads = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[0:3]

        self.FL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_toe), self.FL_loads)
        self.FR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_toe), self.FR_loads)
        self.RL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_toe), self.RL_loads)
        self.RR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_toe), self.RR_loads)

        ###############################################
        ### Calculate Forces and Moments from Tires ###
        ###############################################

        vehicle_centric_forces = self.FL_forces_aligned + self.FR_forces_aligned + self.RL_forces_aligned + self.RR_forces_aligned
        vehicle_centric_moments = np.cross(-1 * self.FL_Cp_wrt_cg, self.FL_forces_aligned) + np.cross(-1 * self.FR_Cp_wrt_cg, self.FR_forces_aligned) + \
            np.cross(-1 * self.RL_Cp_wrt_cg, self.RL_forces_aligned) + np.cross(-1 * self.RR_Cp_wrt_cg, self.RR_forces_aligned)

        # Add gravity
        gravity_forces = np.array([0, 0, -self.vehicle.total_mass * self.vehicle.environment["G"]])
        vehicle_centric_forces += gravity_forces

        ###################################################
        ### Calculate Forces and Moments from Iteration ###
        ###################################################

        # Initialize translational acceleration
        translational_accelerations_IMF = np.array([ax, ay, 0])

        # Sum of forces and moments
        sum_force = self.vehicle.total_mass * translational_accelerations_IMF
        sum_moment = np.dot(self.vehicle.masses["SI"], np.array([0, 0, yaw_ddot]))

        force_residuals = vehicle_centric_forces - sum_force
        moment_residuals = vehicle_centric_moments - sum_moment

        residuals = np.linalg.norm([*force_residuals, *moment_residuals, *Fz_resid])

        return residuals

    def generate_constant_radius_YMD(self, radius: float) -> None:
        pass

    def generate_suspension_report(self) -> None:
        """
        ## Generate Suspension Report

        Saves suspension report to /outputs with the following:
        - Inclination angle vs bump (Bump Camber)
        - Inclination angle vs roll (Roll Camber)
        - Toe angle vs bump (Bump Toe)
        - Toe angle vs roll (Roll Toe)
        - Caster angle vs bump (Bump Caster)
        - Caster angle vs roll (Roll Caster)
        - Mechanical trail vs bump
        - Mechanical trail vs roll
        - Kingpin inclination vs bump
        - Kingpin inclination vs roll
        - Ackermann vs steer
        - Antidive vs Bump
        - Antisquat vs Bump
        - Roll Center Height vs Ay

        Parameters
        ----------
        None
        """
        pass

    def plot(self):

        fig = plt.figure()
        ax = fig.gca()
        ax.set_title("Yaw Acceleration vs Lateral Acceleration")
        ax.set_xlabel("Lateral Acceleration (m/s^2)")
        ax.set_ylabel("Yaw Acceleration (rad/s^2)")
        ax.axhline(c="gray", linewidth=0.5)
        ax.axvline(c="gray", linewidth=0.5)

        mp = self.mesh // 2

        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            ax.plot(lat_accels, yaw_accels, c="red", linewidth=0.8)
            ax.scatter(lat_accels, yaw_accels, s=0.5, c="black")

            # text_pos = ((lat_accels[mp] + lat_accels[mp - 1]) / 2, (yaw_accels[mp] + yaw_accels[mp - 1]) / 2 - 1.0)
            text_pos = (lat_accels[-1] + ((lat_accels[-1] < 0) * (lat_accels[-1] * 0.05 - 0.5)), yaw_accels[-1] + 0.7)
            ax.text(text_pos[0], text_pos[1], f'δ = {round(steered_angle, 1)}°', fontsize=6, c="red")

        for body_slip, lat_accels, yaw_accels in self.body_slip_iso_lines:
            ax.plot(lat_accels, yaw_accels, c="blue", linewidth=0.8)
            a = 0.5 + np.sin(lat_accels[mp] ** 3 / 500) * 0.4
            text_pos = (lat_accels[mp] * a + lat_accels[mp + 1] * (1-a), yaw_accels[mp] * a + yaw_accels[mp + 1] * (1-a) + 0.2)
            # text_pos = (lat_accels[0] + 0.6, yaw_accels[0] + 0.3)
            ax.text(text_pos[0], text_pos[1], f'β = {round(body_slip, 1)}°', fontsize=6, c="blue")

        plt.show()