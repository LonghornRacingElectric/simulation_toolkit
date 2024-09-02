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
                x_ddot, y_ddot, yaw_ddot, heave, pitch, roll = fsolve(self._residual_function, \
                                        x0=[0, 0, 0, 0, 0, 0], args=[delta, beta, velocity])

                self.steered_angle_iso_lines[j][0] = delta / (3.50 / 360 * 0.0254)
                self.steered_angle_iso_lines[i][1][j] = y_ddot
                self.steered_angle_iso_lines[i][2][j] = yaw_ddot
                self.body_slip_iso_lines[i][0] = beta
                self.body_slip_iso_lines[j][1][i] = y_ddot
                self.body_slip_iso_lines[j][2][i] = yaw_ddot
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
        FL_double_wishbone = suspension.Fr_axle.left
        FR_double_wishbone = suspension.Fr_axle.right
        RL_double_wishbone = suspension.Rr_axle.left
        RR_double_wishbone = suspension.Rr_axle.right
        sus_corners: Sequence[DoubleWishbone] = [FL_double_wishbone, FR_double_wishbone, RL_double_wishbone, RR_double_wishbone]

        # Calculate jounce
        self.cg_pos = suspension.cg.position

        # Store contact patch positions
        self.FL_Cp_wrt_cg = FL_double_wishbone.contact_patch.position - self.cg_pos
        self.FR_Cp_wrt_cg = FR_double_wishbone.contact_patch.position - self.cg_pos
        self.RL_Cp_wrt_cg = RL_double_wishbone.contact_patch.position - self.cg_pos
        self.RR_Cp_wrt_cg = RR_double_wishbone.contact_patch.position - self.cg_pos

        # Apply modal displacements
        suspension.steer(rack_displacement=delta)
        suspension.heave(heave=heave)
        suspension.pitch(pitch=pitch * 180 / np.pi)
        suspension.roll(roll=roll * 180 / np.pi)

        # Calculate normal loads
        
        # Inelastic lateral load transfer
        FL_inelastic_lat = FL_double_wishbone.weight / self.vehicle.environment["G"] * \
            FL_double_wishbone.FV_FAP.position[2] * ay / abs(suspension.Fr_axle.track_width)
        FR_inelastic_lat = FR_double_wishbone.weight / self.vehicle.environment["G"] * \
            FR_double_wishbone.FV_FAP.position[2] * ay / abs(suspension.Fr_axle.track_width)
        RL_inelastic_lat = RL_double_wishbone.weight / self.vehicle.environment["G"] * \
            RL_double_wishbone.FV_FAP.position[2] * ay / abs(suspension.Rr_axle.track_width)
        RR_inelastic_lat = RR_double_wishbone.weight / self.vehicle.environment["G"] * \
            RR_double_wishbone.FV_FAP.position[2] * ay / abs(suspension.Rr_axle.track_width)
        
        Fr_inelastic_lat = FL_inelastic_lat + FR_inelastic_lat
        Rr_inelastic_lat = RL_inelastic_lat + RR_inelastic_lat
        
        # Inelastic longitudinal load transfer
        FL_inelastic_long = FL_double_wishbone.weight / self.vehicle.environment["G"] * \
            FL_double_wishbone.SV_FAP.position[2] * ax / abs(suspension.full_suspension.left_wheelbase)
        FR_inelastic_long = FR_double_wishbone.weight / self.vehicle.environment["G"] * \
            FR_double_wishbone.SV_FAP.position[2] * ax / abs(suspension.full_suspension.right_wheelbase)
        RL_inelastic_long = RL_double_wishbone.weight / self.vehicle.environment["G"] * \
            RL_double_wishbone.SV_FAP.position[2] * ax / abs(suspension.full_suspension.left_wheelbase)
        RR_inelastic_long = RR_double_wishbone.weight / self.vehicle.environment["G"] * \
            RR_double_wishbone.SV_FAP.position[2] * ax / abs(suspension.full_suspension.right_wheelbase)
        
        left_inelastic_long = FL_inelastic_long + RL_inelastic_long
        right_inelastic_long = FR_inelastic_long + RR_inelastic_long
        
        # Elastic lateral load transfer
        FL_elastic_lat_inertial = FL_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FL_double_wishbone.FV_FAP.position[2]) * ay
        FR_elastic_lat_inertial = FR_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FR_double_wishbone.FV_FAP.position[2]) * ay
        RL_elastic_lat_inertial = RL_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RL_double_wishbone.FV_FAP.position[2]) * ay
        RR_elastic_lat_inertial = RR_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RR_double_wishbone.FV_FAP.position[2]) * ay
        
        # Fr_elastic_lat_normal = FL_double_wishbone.weight * FL_double_wishbone.lateral_arm - FR_double_wishbone.weight * FR_double_wishbone.lateral_arm
        # Rr_elastic_lat_normal = RL_double_wishbone.weight * RL_double_wishbone.lateral_arm - RR_double_wishbone.weight * RR_double_wishbone.lateral_arm

        Fr_elastic_lat = (FL_elastic_lat_inertial + FR_elastic_lat_inertial) / suspension.Fr_axle.track_width
        Rr_elastic_lat = (RL_elastic_lat_inertial + RR_elastic_lat_inertial) / suspension.Rr_axle.track_width

        total_elastic_lt_lat = Fr_elastic_lat + Rr_elastic_lat

        # Elastic longitudinal load transfer
        FL_elastic_long_inertial = FL_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FL_double_wishbone.SV_FAP.position[2]) * ax
        FR_elastic_long_inertial = FR_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - FR_double_wishbone.SV_FAP.position[2]) * ax
        RL_elastic_long_inertial = RL_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RL_double_wishbone.SV_FAP.position[2]) * ax
        RR_elastic_long_inertial = RR_double_wishbone.weight / self.vehicle.environment["G"] * (self.cg_pos[2] - RR_double_wishbone.SV_FAP.position[2]) * ax
        
        # left_elastic_long_normal = FL_double_wishbone.weight * FL_double_wishbone.longitudinal_arm - RL_double_wishbone.weight * RL_double_wishbone.longitudinal_arm
        # right_elastic_long_normal = FR_double_wishbone.weight * FR_double_wishbone.longitudinal_arm - RR_double_wishbone.weight * RR_double_wishbone.longitudinal_arm
        
        left_elastic_long = (FL_elastic_long_inertial + RL_elastic_long_inertial) / suspension.full_suspension.left_wheelbase
        right_elastic_long = (FR_elastic_long_inertial + RR_elastic_long_inertial) / suspension.full_suspension.right_wheelbase

        total_elastic_lt_long = left_elastic_long + right_elastic_long

        # Total load transfers
        Fr_lat_lt = Fr_inelastic_lat + suspension.Fr_axle.roll_stiffness / (suspension.Fr_axle.roll_stiffness + suspension.Rr_axle.roll_stiffness) * \
            total_elastic_lt_lat
        Rr_lat_lt = Rr_inelastic_lat + suspension.Rr_axle.roll_stiffness / (suspension.Fr_axle.roll_stiffness + suspension.Rr_axle.roll_stiffness) * \
            total_elastic_lt_lat

        left_long_lt = left_inelastic_long + left_elastic_long
        right_long_lt = right_inelastic_long + right_elastic_long
        
        FL_Fz = FL_double_wishbone.weight - Fr_lat_lt + left_long_lt
        FR_Fz = FR_double_wishbone.weight + Fr_lat_lt + right_long_lt
        RL_Fz = RL_double_wishbone.weight - Rr_lat_lt - left_long_lt
        RR_Fz = RR_double_wishbone.weight + Rr_lat_lt - right_long_lt

        Fr_lat_lt = (FL_double_wishbone.weight + FR_double_wishbone.weight) / self.vehicle.environment["G"] * ay * self.cg_pos[2] / suspension.Fr_axle.track_width
        Rr_lat_lt = (RL_double_wishbone.weight + RR_double_wishbone.weight) / self.vehicle.environment["G"] * ay * self.cg_pos[2] / suspension.Rr_axle.track_width

        FL_Fz = FL_double_wishbone.weight - Fr_lat_lt
        FR_Fz = FR_double_wishbone.weight + Fr_lat_lt
        RL_Fz = RL_double_wishbone.weight - Rr_lat_lt
        RR_Fz = RR_double_wishbone.weight + Rr_lat_lt

        print()
        print(FL_Fz)
        print(FR_Fz)
        print(RL_Fz)
        print(RR_Fz)
        print()
        # print()
        # print("Lateral Load Transfers:")
        # print(FL_Fz)
        # print(FR_Fz)
        # print("Longitudinal Load Transfers:")
        # print(RL_Fz)
        # print(RR_Fz)
        # print()

        # Store inclination angles
        FL_gamma = FL_double_wishbone.inclination_angle
        FR_gamma = FR_double_wishbone.inclination_angle
        RL_gamma = RL_double_wishbone.inclination_angle
        RR_gamma = RR_double_wishbone.inclination_angle

        # Store toe angles (static and steered)
        FL_toe = FL_double_wishbone.toe
        FR_toe = FR_double_wishbone.toe
        RL_toe = RL_double_wishbone.toe
        RR_toe = RR_double_wishbone.toe

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

        # Calculate force application points about instant screw axes
        FL_center = plane_eval(points=[FL_double_wishbone.contact_patch.position, FL_double_wishbone.FVIC.position, FL_double_wishbone.SVIC.position],
                               x=FL_double_wishbone.SVIC.position[0],
                               y=FL_double_wishbone.FVIC.position[1])
        FR_center = plane_eval(points=[FR_double_wishbone.contact_patch.position, FR_double_wishbone.FVIC.position, FR_double_wishbone.SVIC.position],
                               x=FR_double_wishbone.SVIC.position[0],
                               y=FR_double_wishbone.FVIC.position[1])
        RL_center = plane_eval(points=[RL_double_wishbone.contact_patch.position, RL_double_wishbone.FVIC.position, RL_double_wishbone.SVIC.position],
                               x=RL_double_wishbone.SVIC.position[0],
                               y=RL_double_wishbone.FVIC.position[1])
        RR_center = plane_eval(points=[RR_double_wishbone.contact_patch.position, RR_double_wishbone.FVIC.position, RR_double_wishbone.SVIC.position],
                               x=RR_double_wishbone.SVIC.position[0],
                               y=RR_double_wishbone.FVIC.position[1])
        # print()
        # print("########")
        # print(FL_center)
        # print(FR_center)
        # print(RL_center)
        # print(RR_center)
        # print("########")
        # print()

        # Sprung mass forces and moments
        sprung_mass_moment = np.cross(FL_center - FL_double_wishbone.contact_patch.position, self.FL_forces_aligned) + \
            np.cross(FR_center - FR_double_wishbone.contact_patch.position, self.FR_forces_aligned) + \
            np.cross(RL_center - RL_double_wishbone.contact_patch.position, self.RL_forces_aligned) + \
            np.cross(RR_center - RR_double_wishbone.contact_patch.position, self.RR_forces_aligned)
        sprung_mass_force = self.FL_forces_aligned + self.FR_forces_aligned + self.RL_forces_aligned + self.RR_forces_aligned

        ###############################################
        ### Calculate Forces and Moments from Tires ###
        ###############################################

        vehicle_centric_forces = self.FL_forces_aligned + self.FR_forces_aligned + self.RL_forces_aligned + self.RR_forces_aligned
        vehicle_centric_moments = np.cross(-1 * self.FL_Cp_wrt_cg, self.FL_forces_aligned) + np.cross(-1 * self.FR_Cp_wrt_cg, self.FR_forces_aligned) + \
            np.cross(-1 * self.RL_Cp_wrt_cg, self.RL_forces_aligned) + np.cross(-1 * self.RR_Cp_wrt_cg, self.RR_forces_aligned)

        # Add gravity
        gravity_forces = np.array([0, 0, -self.vehicle.total_mass * self.vehicle.environment["G"]])
        vehicle_centric_forces += gravity_forces

        ##################################################
        ### Calculate Resultant Heave, Pitch, and Roll ###
        ##################################################

        total_roll_stiffness = suspension.Fr_axle.roll_stiffness + suspension.Rr_axle.roll_stiffness
        total_pitch_stiffness = suspension.full_suspension.pitch_stiffness
        total_heave_stiffness = suspension.full_suspension.heave_stiffness

        calculated_heave = (self.vehicle.SM * self.vehicle.environment["G"] + -1 * sprung_mass_force[2]) / total_heave_stiffness
        calculated_roll = sprung_mass_moment[0] / total_roll_stiffness
        calculated_pitch = sprung_mass_moment[1] / total_pitch_stiffness

        calculated_heave = (self.vehicle.SM * self.vehicle.environment["G"] + -1 * sprung_mass_force[2]) / total_heave_stiffness
        calculated_roll = self.vehicle.total_mass / 9.81 * ay * self.cg_pos[2] / total_roll_stiffness
        calculated_pitch = self.vehicle.total_mass / 9.81 * ax * self.cg_pos[2] / total_pitch_stiffness

        # print()
        # print("###")
        # print(heave)
        # print(calculated_heave)
        # print("###")
        # print(roll)
        # print(calculated_roll)
        # print("###")
        # print(pitch)
        # print(calculated_pitch)
        # print("###")
        # print()

        modal_disp_resid = [heave - calculated_heave, roll - calculated_roll, pitch - calculated_pitch]

        # print("###")
        # print(calculated_roll)
        # print("###")
        # print(calculated_pitch)

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

        print()
        print(np.linalg.norm(modal_disp_resid))
        print()

        residuals = np.array([*force_residuals, *moment_residuals]) + np.linalg.norm(modal_disp_resid)

        # print(residuals)

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
        plt.title("Yaw Acceleration vs Lateral Acceleration")
        plt.xlabel("Lateral Acceleration (m/s^2)")
        plt.ylabel("Yaw Acceleration (rad/s^2)")
        plt.axhline(c="gray", linewidth=0.5)
        plt.axvline(c="gray", linewidth=0.5)

        mp = self.mesh // 2

        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="red", linewidth=0.8)
            plt.scatter(lat_accels, yaw_accels, s=0.5, c="black")

            # text_pos = ((lat_accels[mp] + lat_accels[mp - 1]) / 2, (yaw_accels[mp] + yaw_accels[mp - 1]) / 2 - 1.0)
            text_pos = (lat_accels[-1] + ((lat_accels[-1] < 0) * (lat_accels[-1] * 0.05 - 0.5)), yaw_accels[-1] + 0.7)
            plt.text(text_pos[0], text_pos[1], f'δ = {round(steered_angle, 1)}°', fontsize=6, c="red")

        for body_slip, lat_accels, yaw_accels in self.body_slip_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="blue", linewidth=0.8)
            a = 0.5 + np.sin(lat_accels[mp] ** 3 / 500) * 0.4
            text_pos = (lat_accels[mp] * a + lat_accels[mp + 1] * (1-a), yaw_accels[mp] * a + yaw_accels[mp + 1] * (1-a) + 0.2)
            # text_pos = (lat_accels[0] + 0.6, yaw_accels[0] + 0.3)
            plt.text(text_pos[0], text_pos[1], f'β = {round(body_slip, 1)}°', fontsize=6, c="blue")

        plt.show()