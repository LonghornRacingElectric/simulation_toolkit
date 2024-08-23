from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_model import SuspensionModel
from vehicle_model._assets.misc_linalg import rotation_matrix
from vehicle_model._assets.misc_linalg import plane_eval
from vehicle_model.vehicle_model import VehicleModel
from LHR_tire_toolkit.MF52 import MF52
from typing import Sequence, Tuple
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import numpy as np
import time


class YMD:
    def __init__(self, vehicle: VehicleModel, mesh: float) -> None:
        self.vehicle = vehicle
        self.mesh = mesh

        self.delta_sweep = np.linspace(-15 * 3.50 / 360 * 0.0254, 15 * 3.50 / 360 * 0.0254, mesh)
        self.beta_sweep = np.linspace(-10 * np.pi / 180, 10 * np.pi / 180, mesh)
    
    def generate_constant_velocity_YMD(self, velocity: float) -> None:
        self.body_slip_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.steered_angle_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.all_points = []

        total_states = len(self.delta_sweep) * len(self.beta_sweep)
        counter = 0
        total_start = time.time()

        y_ddot_lst = []
        yaw_ddot_lst = []

        for i, beta in enumerate(self.beta_sweep):
            for j, delta in enumerate(self.delta_sweep):
                print(f"Progress | {round(counter / total_states * 100, 1)}%", end="\r")
                counter += 1
                x_ddot, y_ddot, yaw_ddot, heave, pitch, roll = fsolve(self._residual_function, \
                                        x0=[0, 0, 0, 0, 0, 0], args=[delta, beta, velocity])
                
                self.steered_angle_iso_lines[j][0] = delta
                self.steered_angle_iso_lines[i][1][j] = y_ddot
                self.steered_angle_iso_lines[i][2][j] = yaw_ddot
                self.body_slip_iso_lines[i][0] = beta
                self.body_slip_iso_lines[j][1][i] = y_ddot
                self.body_slip_iso_lines[j][2][i] = yaw_ddot

                y_ddot_lst.append(y_ddot)
                yaw_ddot_lst.append(yaw_ddot)
        
        total_end = time.time()

        print(f"Total Runtime: {total_end - total_start}")

        self.plot()
        # plt.scatter(y_ddot_lst, yaw_ddot_lst)
        # plt.show()

        # return [heave / 0.0254, pitch * 180 / np.pi, roll * 180 / np.pi]

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

        start_iter = time.time()
        ax = x[0]
        ay = x[1]
        yaw_ddot = x[2]
        heave = x[3]
        pitch = x[4]
        roll = x[5]

        if str(heave) == 'nan' or str(pitch) == 'nan' or str(roll) == 'nan':
            return [1e9, 1e9, 1e9, 1e9, 1e9, 1e9]

        # print(heave / 0.0254, pitch * 180 / np.pi, roll * 180 / np.pi)

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

        # Store contact patch positions
        self.FL_Cp_pos = FL_double_wishbone.contact_patch.position
        self.FR_Cp_pos = FR_double_wishbone.contact_patch.position
        self.RL_Cp_pos = RL_double_wishbone.contact_patch.position
        self.RR_Cp_pos = RR_double_wishbone.contact_patch.position

        # Calculate jounce
        cg_pos = suspension.cg.position
        
        FL_jounce = heave + abs(self.FL_Cp_pos[0] - cg_pos[0]) * np.tan(pitch) + -1 * abs(self.FL_Cp_pos[1] - cg_pos[1]) * np.tan(roll)
        FR_jounce = heave + abs(self.FR_Cp_pos[0] - cg_pos[0]) * np.tan(pitch) + abs(self.FR_Cp_pos[1] - cg_pos[1]) * np.tan(roll)
        RL_jounce = heave + -1 * abs(self.RL_Cp_pos[0] - cg_pos[0]) * np.tan(pitch) + -1 * abs(self.RL_Cp_pos[1] - cg_pos[1]) * np.tan(roll)
        RR_jounce = heave + -1 * abs(self.RR_Cp_pos[0] - cg_pos[0]) * np.tan(pitch) + abs(self.RR_Cp_pos[1] - cg_pos[1]) * np.tan(roll)
        sus_corners_jounce: Sequence[float] = [FL_jounce, FR_jounce, RL_jounce, RR_jounce]
        # print([x / 0.0254 for x in [FL_jounce, FR_jounce, RL_jounce, RR_jounce]])

        # Inclination angles
        FL_gamma = suspension.FL_gamma_lookup(x=delta, y=FL_jounce)[0] + roll
        FR_gamma = suspension.FR_gamma_lookup(x=delta, y=FR_jounce)[0] + roll
        RL_gamma = suspension.RL_gamma_lookup(x=0, y=RL_jounce)[0] + roll
        RR_gamma = suspension.RR_gamma_lookup(x=0, y=RR_jounce)[0] + roll

        # Toe angles
        FL_toe = suspension.FL_toe_lookup(x=delta, y=FL_jounce)[0] * -1
        FR_toe = suspension.FR_toe_lookup(x=delta, y=FR_jounce)[0] * -1
        RL_toe = suspension.RL_toe_lookup(x=0, y=RL_jounce)[0] * -1
        RR_toe = suspension.RR_toe_lookup(x=0, y=RR_jounce)[0] * -1
        # print(FL_toe * 180 / np.pi)
        # print(FR_toe * 180 / np.pi)
        # print(RL_toe * 180 / np.pi)
        # print(RR_toe * 180 / np.pi)

        # Wheel velocities
        FL_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.FL_Cp_pos)
        FR_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.FR_Cp_pos)
        RL_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.RL_Cp_pos)
        RR_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.RR_Cp_pos)

        # Slip angles
        FL_alpha = FL_toe - np.arctan2(FL_velocity[1], FL_velocity[0])
        FR_alpha = FR_toe - np.arctan2(FR_velocity[1], FR_velocity[0])
        RL_alpha = RL_toe - np.arctan2(RL_velocity[1], RL_velocity[0])
        RR_alpha = RR_toe - np.arctan2(RR_velocity[1], RR_velocity[0])

        # print([x * 180 / np.pi for x in [FL_alpha, FR_alpha, RL_alpha, RR_alpha]])

        # This is jank, but find real solution for modal displacements
        # x[3], x[4], x[5] = fsolve(self._IC_residual_function, x0=[heave, pitch, roll], args=[[imf_velocity, yaw_rate], [FL_tire, FR_tire, RL_tire, RR_tire], suspension])

        # self.heave_soln = x[3]
        # self.pitch_soln = x[4]
        # self.roll_soln = x[5]

        # Normal loads
        Fz_lst = []
        for i in range(len(sus_corners)):
            if sus_corners_jounce[i] >= 0:
                Fz_lst.append(sus_corners[i].weight + sus_corners[i].wheelrate_function.integrate(0, sus_corners_jounce[i]))
            else:
                Fz_lst.append(sus_corners[i].weight - sus_corners[i].wheelrate_function.integrate(sus_corners_jounce[i], 0))

        if Fz_lst[2] == 0 and Fz_lst[3] == 0:
            Fz_lst[0] += Fz_lst[2]
            Fz_lst[1] += Fz_lst[3]
            Fz_lst[2] = 0
            Fz_lst[3] = 0

        FL_Fz, FR_Fz, RL_Fz, RR_Fz = Fz_lst

        # Tire loads
        self.FL_loads = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[0:3]
        self.FR_loads = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[0:3]
        self.RL_loads = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[0:3]
        self.RR_loads = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[0:3]

        self.FL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_toe), self.FL_loads)
        self.FR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_toe), self.FR_loads)
        self.RL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_toe), self.RL_loads)
        self.RR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_toe), self.RR_loads)

        # print(self.FL_forces_aligned)
        # print(self.FR_forces_aligned)
        # print(self.RL_forces_aligned)
        # print(self.RR_forces_aligned)

        ###############################################
        ### Calculate Forces and Moments from Tires ###
        ###############################################

        vehicle_centric_forces = self.FL_forces_aligned + self.FR_forces_aligned + self.RL_forces_aligned + self.RR_forces_aligned
        vehicle_centric_moments = np.cross(self.FL_Cp_pos, self.FL_forces_aligned) + np.cross(self.FR_Cp_pos, self.FR_forces_aligned) + \
            np.cross(self.RL_Cp_pos, self.RL_forces_aligned) + np.cross(self.RR_Cp_pos, self.RR_forces_aligned)
        
        # print(vehicle_centric_forces)

        # Add gravity
        gravity_forces = np.array([0, 0, -self.vehicle.total_mass * self.vehicle.environment["G"]])
        # print(gravity_forces)
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

        # print([*force_residuals, *moment_residuals])

        end_iter = time.time()

        # print(f"Complete State: {end_iter - start_iter}")

        return [*force_residuals, *moment_residuals]

    def _IC_residual_function(self, x: Sequence[float], args: Tuple[Tuple[np.ndarray, float], Sequence[MF52], SuspensionModel]) -> Sequence[float]:
        """
        ## Anti Residual Function

        Checks for convergence between heave, pitch, and roll

        Parameters
        ----------
        x : Sequence[float]
            Solution guess in the form: [heave, pitch, roll]
        args : Sequence[SuspensionModel]
            Args in the form: [imf_velocity_x, imf_velocity_y, yaw_rate, FL_tire, FR_tire, RL_tire, RR_tire, SuspensionModel]
        
        Returns
        -------
        Sequence[float]
            Residuals
        """
        start_iter = time.time()
        # Initialize solution guess
        heave = x[0]
        pitch = x[1]
        roll = x[2]
        
        # Initialize args
        imf_velocity, yaw_rate = args[0]
        FL_tire, FR_tire, RL_tire, RR_tire = args[1]
        suspension = args[2]

        Fr_axle = suspension.Fr_axle
        Rr_axle = suspension.Rr_axle
        FL_double_wishbone = Fr_axle.left
        FR_double_wishbone = Fr_axle.right
        RL_double_wishbone = Rr_axle.left
        RR_double_wishbone = Rr_axle.right

        # Apply modal suspension displacements
        # previous_heave = self.vehicle.suspension.current_heave
        # previous_pitch = self.vehicle.suspension.current_pitch
        # previous_roll = self.vehicle.suspension.current_roll
        # if abs((heave - previous_heave) / previous_heave) > 0.25:
        #     self.vehicle.suspension.heave(heave=heave)
        # if abs((pitch - previous_pitch) / previous_pitch) > 0.25:
        #     self.vehicle.suspension.pitch(pitch=pitch)
        # if abs((roll - previous_roll) / previous_roll) > 0.25:
        #     self.vehicle.suspension.roll(roll=roll)

        #############################
        ### Calculate Tire Forces ###
        #############################

        # Pull contact patch locations
        self.FL_Cp = FL_double_wishbone.contact_patch
        self.FR_Cp = FR_double_wishbone.contact_patch
        self.RL_Cp = RL_double_wishbone.contact_patch
        self.RR_Cp = RR_double_wishbone.contact_patch

        # Pull jounce values
        FL_jounce = FL_double_wishbone.total_jounce
        FR_jounce = FR_double_wishbone.total_jounce
        RL_jounce = RL_double_wishbone.total_jounce
        RR_jounce = RR_double_wishbone.total_jounce

        # Calculate Normal loads
        FL_Fz = FL_double_wishbone.weight + FL_double_wishbone.wheelrate_function.integrate(0, FL_jounce)
        FR_Fz = FR_double_wishbone.weight + FR_double_wishbone.wheelrate_function.integrate(0, FR_jounce)
        RL_Fz = RL_double_wishbone.weight + RL_double_wishbone.wheelrate_function.integrate(0, RL_jounce)
        RR_Fz = RR_double_wishbone.weight + RR_double_wishbone.wheelrate_function.integrate(0, RR_jounce)
        
        if FL_Fz <= 0 or FR_Fz <= 0 or RL_Fz <= 0 or RR_Fz <= 0:
            return [1e9, 1e9, 1e9]

        # Slip angles
        FL_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.FL_Cp.position)
        FR_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.FR_Cp.position)
        RL_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.RL_Cp.position)
        RR_velocity = imf_velocity + np.cross(np.array([0, 0, yaw_rate]), self.RR_Cp.position)

        self.FL_toe = FL_double_wishbone.toe
        self.FR_toe = FR_double_wishbone.toe
        self.RL_toe = RL_double_wishbone.toe
        self.RR_toe = RR_double_wishbone.toe

        FL_alpha = self.FL_toe - np.arctan2(FL_velocity[1], FL_velocity[0])
        FR_alpha = self.FR_toe - np.arctan2(FR_velocity[1], FR_velocity[0])
        RL_alpha = self.RL_toe - np.arctan2(RL_velocity[1], RL_velocity[0])
        RR_alpha = self.RR_toe - np.arctan2(RR_velocity[1], RR_velocity[0])

        # Inclination angles
        FL_gamma = FL_double_wishbone.inclination_angle
        FR_gamma = FR_double_wishbone.inclination_angle
        RL_gamma = RL_double_wishbone.inclination_angle
        RR_gamma = RR_double_wishbone.inclination_angle

        # Tire loads
        self.FL_loads = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[0:3]
        self.FR_loads = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[0:3]
        self.RL_loads = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[0:3]
        self.RR_loads = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[0:3]

        self.FL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=self.FL_toe), self.FL_loads)
        self.FR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=self.FR_toe), self.FR_loads)
        self.RL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=self.RL_toe), self.RL_loads)
        self.RR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=self.RR_toe), self.RR_loads)

        # Calculate force application points
        FL_center = plane_eval(points=[FL_double_wishbone.contact_patch.position, FL_double_wishbone.FVIC.position, FL_double_wishbone.SVIC.position],
                               x=FL_double_wishbone.SVIC.position,
                               y=FL_double_wishbone.FVIC.position)
        FR_center = plane_eval(points=[FR_double_wishbone.contact_patch.position, FR_double_wishbone.FVIC.position, FR_double_wishbone.SVIC.position],
                               x=FR_double_wishbone.SVIC.position,
                               y=FR_double_wishbone.FVIC.position)
        RL_center = plane_eval(points=[RL_double_wishbone.contact_patch.position, RL_double_wishbone.FVIC.position, RL_double_wishbone.SVIC.position],
                               x=RL_double_wishbone.SVIC.position,
                               y=RL_double_wishbone.FVIC.position)
        RR_center = plane_eval(points=[RR_double_wishbone.contact_patch.position, RR_double_wishbone.FVIC.position, RR_double_wishbone.SVIC.position],
                               x=RR_double_wishbone.SVIC.position,
                               y=RR_double_wishbone.FVIC.position)

        # Calculate forces and moments on sprung mass
        FL_moment_arm = Fr_axle.cg.position - FL_center
        FR_moment_arm = Fr_axle.cg.position - FR_center
        RL_moment_arm = Rr_axle.cg.position - RL_center
        RR_moment_arm = Rr_axle.cg.position - RR_center

        F_total = self.FL_loads + self.FR_loads + self.RL_loads + self.RR_loads

        M_total = np.cross(FL_moment_arm, self.FL_forces_aligned)[0] + np.cross(FR_moment_arm, self.FR_forces_aligned)[0] + \
            np.cross(RL_moment_arm, self.RL_forces_aligned)[0] + np.cross(RR_moment_arm, self.RR_forces_aligned)[0]
        
        M_roll = M_total[0]
        M_pitch = M_total[1]

        roll_stiffness = suspension.full_suspension.roll_stiffness
        calculated_roll = M_roll / roll_stiffness

        pitch_stiffness = suspension.full_suspension.pitch_stiffness
        calculated_pitch = M_pitch / pitch_stiffness

        heave_stiffness = suspension.full_suspension.heave_stiffness
        calculated_heave = F_total[2] / heave_stiffness

        end_iter = time.time()

        # print(f"Suspension State Total Time: {end_iter - start_iter}")

        return [heave - calculated_heave, pitch - calculated_pitch, roll - calculated_roll]

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