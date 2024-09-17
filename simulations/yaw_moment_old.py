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
                
                self.steered_angle_iso_lines[j][0] = beta
                self.steered_angle_iso_lines[i][1][j] = y_ddot
                self.steered_angle_iso_lines[i][2][j] = yaw_ddot \
                # / (self.vehicle.total_mass * 9.81 * abs(self.vehicle.suspension.FL_double_wishbone.contact_patch.position[0] - self.vehicle.suspension.RL_double_wishbone.contact_patch.position[0]))
                self.body_slip_iso_lines[i][0] = delta / (3.50 / 360 * 0.0254)
                self.body_slip_iso_lines[j][1][i] = y_ddot
                self.body_slip_iso_lines[j][2][i] = yaw_ddot \
                #  / (self.vehicle.total_mass * 9.81 * abs(self.vehicle.suspension.FL_double_wishbone.contact_patch.position[0] - self.vehicle.suspension.RL_double_wishbone.contact_patch.position[0]))

                y_ddot_lst.append(y_ddot)
                yaw_ddot_lst.append(yaw_ddot)
                if beta == 0:
                    delta_lst.append(delta / (3.50 / 360 * 0.0254))
                    Ay_lst.append(y_ddot / 9.81)

        plt.plot(Ay_lst, delta_lst)
        plt.show()
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
        
        suspension = self.vehicle.suspension
        self.import_values(suspension=suspension, delta=delta, heave=heave, pitch=pitch, roll=roll)

        #############
        ### Setup ###
        #############

        # Adjust velocity vectors
        imf_velocity = velocity * np.array([np.cos(beta), np.sin(beta), 0])

        # Adjust acceleration vectors and dependencies
        imf_accel = np.array([ax, ay, 0])
        ntb_accel = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=beta), imf_accel)
        yaw_rate = 0 if ay == 0 else ntb_accel[1] / np.linalg.norm(imf_velocity)

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

        self.cg_pos = suspension.cg.position

        # Store contact patch positions
        self.FL_Cp_wrt_cg = FL_double_wishbone.contact_patch.position - self.cg_pos
        self.FR_Cp_wrt_cg = FR_double_wishbone.contact_patch.position - self.cg_pos
        self.RL_Cp_wrt_cg = RL_double_wishbone.contact_patch.position - self.cg_pos
        self.RR_Cp_wrt_cg = RR_double_wishbone.contact_patch.position - self.cg_pos
        
        FL_jounce = heave + abs(self.FL_Cp_wrt_cg[0]) * np.tan(pitch) + -1 * abs(self.FL_Cp_wrt_cg[1]) * np.tan(roll)
        FR_jounce = heave + abs(self.FR_Cp_wrt_cg[0]) * np.tan(pitch) + abs(self.FR_Cp_wrt_cg[1]) * np.tan(roll)
        RL_jounce = heave + -1 * abs(self.RL_Cp_wrt_cg[0]) * np.tan(pitch) + -1 * abs(self.RL_Cp_wrt_cg[1]) * np.tan(roll)
        RR_jounce = heave + -1 * abs(self.RR_Cp_wrt_cg[0]) * np.tan(pitch) + abs(self.RR_Cp_wrt_cg[1]) * np.tan(roll)
        sus_corners_jounce: Sequence[float] = [FL_jounce, FR_jounce, RL_jounce, RR_jounce]

        # Inclination angles
        FL_gamma = self.vehicle.suspension.FL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_gamma = self.vehicle.suspension.FR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_gamma = self.vehicle.suspension.RL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_gamma = self.vehicle.suspension.RR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # Toe angles
        FL_toe = self.vehicle.suspension.FL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_toe = self.vehicle.suspension.FR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_toe = self.vehicle.suspension.RL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_toe = self.vehicle.suspension.RR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

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

        # Normal loads from springs
        Fz_lst = []
        elastic_LT_ratios = {"FL": (self.cg_pos[2] - self.FL_FV_FAPz) / self.cg_pos[2],
                             "FR": (self.cg_pos[2] - self.FR_FV_FAPz) / self.cg_pos[2],
                             "RL": (self.cg_pos[2] - self.RL_FV_FAPz) / self.cg_pos[2],
                             "RR": (self.cg_pos[2] - self.RR_FV_FAPz) / self.cg_pos[2]}
        for i, key in enumerate(elastic_LT_ratios.keys()):
            # Calculate heave, pitch, and roll separately
            # Use ICs to adjust pitch and roll for inelastic LT
            # Use direct jounce to solve for aero
            if sus_corners_jounce[i] >= 0:
                Fz_lst.append(sus_corners[i].weight + sus_corners[i].wheelrate_function.integrate(0, sus_corners_jounce[i]) / abs(elastic_LT_ratios[key]))
            else:
                Fz_lst.append(sus_corners[i].weight - sus_corners[i].wheelrate_function.integrate(sus_corners_jounce[i], 0) / abs(elastic_LT_ratios[key]))
        
        FL_Fz, FR_Fz, RL_Fz, RR_Fz = Fz_lst
        
        # Normal loads from accels
        Fr_lat_LT = (FL_double_wishbone.weight + FR_double_wishbone.weight) * ay / 9.81 * self.cg_pos[2] / abs(self.FL_dw_cp_pos[1] - self.FR_dw_cp_pos[1])
        Rr_lat_LT = (RL_double_wishbone.weight + RR_double_wishbone.weight) * ay / 9.81 * self.cg_pos[2] / abs(self.RL_dw_cp_pos[1] - self.RR_dw_cp_pos[1])
        left_long_LT = self.cg_pos[2] / abs(self.FL_dw_cp_pos[0] - self.RL_dw_cp_pos[0]) * (FL_double_wishbone.weight + RL_double_wishbone.weight) * ax / 9.81
        right_long_LT = self.cg_pos[2] / abs(self.FR_dw_cp_pos[0] - self.RR_dw_cp_pos[0]) * (FR_double_wishbone.weight + RR_double_wishbone.weight) * ax / 9.81
        
        FL_Fz_LT = FL_double_wishbone.weight - Fr_lat_LT - left_long_LT
        FR_Fz_LT = FR_double_wishbone.weight + Fr_lat_LT - right_long_LT
        RL_Fz_LT = RL_double_wishbone.weight - Rr_lat_LT + left_long_LT
        RR_Fz_LT = RR_double_wishbone.weight + Rr_lat_LT + right_long_LT
        Fz_lst_LT = np.array([FL_Fz_LT, FR_Fz_LT, RL_Fz_LT, RR_Fz_LT])

        Fz_resid = Fz_lst_LT - np.array(Fz_lst)

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
        vehicle_centric_moments = np.cross(-1 * self.FL_Cp_wrt_cg, self.FL_forces_aligned) + np.cross(-1 * self.FR_Cp_wrt_cg, self.FR_forces_aligned) + \
            np.cross(-1 * self.RL_Cp_wrt_cg, self.RL_forces_aligned) + np.cross(-1 * self.RR_Cp_wrt_cg, self.RR_forces_aligned)
        
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

        # residuals = np.array([*force_residuals, *moment_residuals]) + np.linalg.norm(Fz_resid)

        # print(residuals)

        # return residuals
        # return np.linalg.norm([*force_residuals, *moment_residuals, *Fz_resid])
        return np.linalg.norm([*force_residuals, *moment_residuals])

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
            # plt.text(text_pos[0], text_pos[1], f'δ = {round(steered_angle, 1)}°', fontsize=6, c="red")

        for body_slip, lat_accels, yaw_accels in self.body_slip_iso_lines:
            plt.plot(lat_accels, yaw_accels, c="blue", linewidth=0.8)
            a = 0.5 + np.sin(lat_accels[mp] ** 3 / 500) * 0.4
            text_pos = (lat_accels[mp] * a + lat_accels[mp + 1] * (1-a), yaw_accels[mp] * a + yaw_accels[mp + 1] * (1-a) + 0.2)
            # text_pos = (lat_accels[0] + 0.6, yaw_accels[0] + 0.3)
            # plt.text(text_pos[0], text_pos[1], f'β = {round(body_slip, 1)}°', fontsize=6, c="blue")

        plt.show()

    def import_values(self, suspension: SuspensionModel, delta: float, heave: float, pitch: float, roll: float) -> None:
        # Import all relevant values
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
        
        cgx = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgy = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgz = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        
        # Force application points
        self.FL_FV_FAPz = suspension.FL_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.FR_FV_FAPz = suspension.FR_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.RL_FV_FAPz = suspension.RL_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.RR_FV_FAPz = suspension.RR_FV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        self.FL_SV_FAPz = suspension.FL_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.FR_SV_FAPz = suspension.FR_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.RL_SV_FAPz = suspension.RL_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.RR_SV_FAPz = suspension.RR_SV_FAPz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        
        # Roll stiffnesses
        self.Fr_Kr = suspension.Fr_Kr_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        self.Rr_Kr = suspension.Rr_Kr_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # Pitch stiffness
        self.Kp = suspension.Kp_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # # Store corner jounce
        # self.FL_jounce = suspension.FL_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.FR_jounce = suspension.FR_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.RL_jounce = suspension.RL_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.RR_jounce = suspension.RR_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        
        # # Store tire objects
        # self.FL_tire = suspension.Fr_axle.left.tire
        # self.FR_tire = suspension.Fr_axle.right.tire
        # self.RL_tire = suspension.Rr_axle.left.tire
        # self.RR_tire = suspension.Rr_axle.right.tire

        # # Store suspension corner objects
        # self.FL_weight = suspension.Fr_axle.left.weight
        # self.FR_weight = suspension.Fr_axle.right.weight
        # self.RL_weight = suspension.Rr_axle.left.weight
        # self.RR_weight = suspension.Rr_axle.right.weight

        # Store cg position
        self.cg_pos = np.array([cgx, cgy, cgz])
        
        # Store contact patch positions
        self.FL_dw_cp_pos = np.array([FL_dw_cpx, FL_dw_cpy, FL_dw_cpz])
        self.FR_dw_cp_pos = np.array([FR_dw_cpx, FR_dw_cpy, FR_dw_cpz])
        self.RL_dw_cp_pos = np.array([RL_dw_cpx, RL_dw_cpy, RL_dw_cpz])
        self.RR_dw_cp_pos = np.array([RR_dw_cpx, RR_dw_cpy, RR_dw_cpz])

        # # Tire shit
        # self.FL_gamma = suspension.FL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.FR_gamma = suspension.FR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.RL_gamma = suspension.RL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.RR_gamma = suspension.RR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # self.FL_toe = suspension.FL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.FR_toe = suspension.FR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.RL_toe = suspension.RL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        # self.RR_toe = suspension.RR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
