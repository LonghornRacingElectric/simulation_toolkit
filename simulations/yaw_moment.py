from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_model import SuspensionModel
from vehicle_model._assets.misc_linalg import rotation_matrix
from vehicle_model.vehicle_model import VehicleModel
from LHR_tire_toolkit.MF52 import MF52
from typing import Sequence, Tuple
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time


class YMD:
    def __init__(self, vehicle: VehicleModel, mesh: float) -> None:
        self.vehicle = vehicle
        self.mesh = mesh

        self.delta_sweep = np.linspace(-35 * 3.50 / 360 * 0.0254, 25 * 3.50 / 360 * 0.0254, mesh)
        self.beta_sweep = np.linspace(-12 * np.pi / 180, 12 * np.pi / 180, mesh)

        self.suspension = self.vehicle.suspension
    
    def generate_constant_velocity_YMD(self, velocity: float) -> None:
        self.body_slip_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.steered_angle_iso_lines = [[0, [0] * self.mesh, [0] * self.mesh] for _ in range(self.mesh)]
        self.all_points = []

        total_states = len(self.delta_sweep) * len(self.beta_sweep)
        counter = 0
        total_start = time.time()

        x_ddot_lst = []
        y_ddot_lst = []
        yaw_ddot_lst = []
        heave_lst = []
        pitch_lst = []
        roll_lst = []

        delta_lst = []
        beta_lst = []

        for i, beta in enumerate(self.beta_sweep):
            for j, delta in enumerate(self.delta_sweep):
                print(f"Progress | {round(counter / total_states * 100, 1)}%", end="\r")
                counter += 1
                x_ddot, y_ddot, yaw_ddot, heave, pitch, roll = fsolve(self._residual_function, x0=[0, 0, 0, 0, 0, 0], args=[delta, beta, velocity])
                
                x_ddot_lst.append(x_ddot)
                y_ddot_lst.append(y_ddot)
                yaw_ddot_lst.append(yaw_ddot)
                heave_lst.append(heave)
                pitch_lst.append(pitch)
                roll_lst.append(roll)

                delta_lst.append(delta)
                beta_lst.append(beta)

                self.steered_angle_iso_lines[j][0] = delta / (3.50 / (2 * np.pi) * 0.0254)
                self.steered_angle_iso_lines[j][1][i] = y_ddot
                self.steered_angle_iso_lines[j][2][i] = yaw_ddot
                
                self.body_slip_iso_lines[i][0] = beta
                self.body_slip_iso_lines[i][1][j] = y_ddot
                self.body_slip_iso_lines[i][2][j] = yaw_ddot

        max_y_ddot = max(y_ddot_lst)
        corresp_yaw_ddot = yaw_ddot_lst[y_ddot_lst.index(max(y_ddot_lst))]

        print(f"Max y_ddot: {max_y_ddot}")
        print(f"Corresponding yaw_ddot: {corresp_yaw_ddot}")

        Fx_res_lst = []
        Fy_res_lst = []
        Fz_res_lst = []
        Mx_res_lst = []
        My_res_lst = []
        Mz_res_lst = []
        FL_jounce_lst = []
        FR_jounce_lst = []
        RL_jounce_lst = []
        RR_jounce_lst = []
        FL_gamma_lst = []
        FR_gamma_lst = []
        RL_gamma_lst = []
        RR_gamma_lst = []
        FL_alpha_lst = []
        FR_alpha_lst = []
        RL_alpha_lst = []
        RR_alpha_lst = []
        FL_Fx_aligned_lst = []
        FL_Fy_aligned_lst = []
        FL_Fz_aligned_lst = []
        FR_Fx_aligned_lst = []
        FR_Fy_aligned_lst = []
        FR_Fz_aligned_lst = []
        RL_Fx_aligned_lst = []
        RL_Fy_aligned_lst = []
        RL_Fz_aligned_lst = []
        RR_Fx_aligned_lst = []
        RR_Fy_aligned_lst = []
        RR_Fz_aligned_lst = []
        new_heave_lst = []
        new_pitch_lst = []
        new_roll_lst = []
        new_delta_lst = []
        new_beta_lst = []
        ax_lst = []
        ay_lst = []
        new_yaw_ddot_lst = []
        FL_cp_pos_lst = []
        FR_cp_pos_lst = []
        RL_cp_pos_lst = []
        RR_cp_pos_lst = []

        # for pair in zip(x_ddot_lst, y_ddot_lst, yaw_ddot_lst, heave_lst, pitch_lst, roll_lst, delta_lst, beta_lst):
        #     output = self._residual_eval(x = pair[:6], args = [*pair[6:], 25])

        #     Fx_res_lst.append(output[0])
        #     Fy_res_lst.append(output[1])
        #     Fz_res_lst.append(output[2])
        #     Mx_res_lst.append(output[3])
        #     My_res_lst.append(output[4])
        #     Mz_res_lst.append(output[5])
        #     FL_jounce_lst.append(output[6])
        #     FR_jounce_lst.append(output[7])
        #     RL_jounce_lst.append(output[8])
        #     RR_jounce_lst.append(output[9])
        #     FL_gamma_lst.append(output[10])
        #     FR_gamma_lst.append(output[11])
        #     RL_gamma_lst.append(output[12])
        #     RR_gamma_lst.append(output[13])
        #     FL_alpha_lst.append(output[14])
        #     FR_alpha_lst.append(output[15])
        #     RL_alpha_lst.append(output[16])
        #     RR_alpha_lst.append(output[17])
        #     FL_Fx_aligned_lst.append(output[18])
        #     FL_Fy_aligned_lst.append(output[19])
        #     FL_Fz_aligned_lst.append(output[20])
        #     FR_Fx_aligned_lst.append(output[21])
        #     FR_Fy_aligned_lst.append(output[22])
        #     FR_Fz_aligned_lst.append(output[23])
        #     RL_Fx_aligned_lst.append(output[24])
        #     RL_Fy_aligned_lst.append(output[25])
        #     RL_Fz_aligned_lst.append(output[26])
        #     RR_Fx_aligned_lst.append(output[27])
        #     RR_Fy_aligned_lst.append(output[28])
        #     RR_Fz_aligned_lst.append(output[29])
        #     new_heave_lst.append(output[30])
        #     new_pitch_lst.append(output[31])
        #     new_roll_lst.append(output[32])
        #     new_delta_lst.append(output[33])
        #     new_beta_lst.append(output[34])
        #     ax_lst.append(output[35])
        #     ay_lst.append(output[36])
        #     new_yaw_ddot_lst.append(output[37])
        #     FL_cp_pos_lst.append(output[38])
        #     FR_cp_pos_lst.append(output[39])
        #     RL_cp_pos_lst.append(output[40])
        #     RR_cp_pos_lst.append(output[41])

        # output_dict = {"x_ddot": ax_lst,
        #                "y_ddot": ay_lst,
        #                "yaw_ddot": yaw_ddot_lst,
        #                "delta": [x * 180 / np.pi / (3.50 / (2 * np.pi) * 0.0254) for x in delta_lst],
        #                "beta": [x * 180 / np.pi for x in beta_lst],
        #                "FL_cp_pos": FL_cp_pos_lst,
        #                "FR_cp_pos": FR_cp_pos_lst,
        #                "RL_cp_pos": RL_cp_pos_lst,
        #                "RR_cp_pos": RR_cp_pos_lst,
        #                "FL_SA": [x * 180 / np.pi for x in FL_alpha_lst],
        #                "FR_SA": [x * 180 / np.pi for x in FR_alpha_lst],
        #                "RL_SA": [x * 180 / np.pi for x in RL_alpha_lst],
        #                "RR_SA": [x * 180 / np.pi for x in RR_alpha_lst],
        #                "heave": [x / 0.0254 for x in heave_lst],
        #                "pitch": [x * 180 / np.pi for x in pitch_lst],
        #                "roll": [x * 180 / np.pi for x in roll_lst],
        #                "FL jounce": [x / 0.0254 for x in FL_jounce_lst],
        #                "FR jounce": [x / 0.0254 for x in FR_jounce_lst],
        #                "RL jounce": [x / 0.0254 for x in RL_jounce_lst],
        #                "RR jounce": [x / 0.0254 for x in RR_jounce_lst],
        #                "FL gamma": [x * 180 / np.pi for x in FL_gamma_lst],
        #                "FR gamma": [x * 180 / np.pi for x in FR_gamma_lst],
        #                "RL gamma": [x * 180 / np.pi for x in RL_gamma_lst],
        #                "RR gamma": [x * 180 / np.pi for x in RR_gamma_lst],
        #                "FL_Fx": FL_Fx_aligned_lst,
        #                "FL_Fy": FL_Fy_aligned_lst,
        #                "FL_Fz": FL_Fz_aligned_lst,
        #                "FR_Fx": FR_Fx_aligned_lst,
        #                "FR_Fy": FR_Fy_aligned_lst,
        #                "FR_Fz": FR_Fz_aligned_lst,
        #                "RL_Fx": RL_Fx_aligned_lst,
        #                "RL_Fy": RL_Fy_aligned_lst,
        #                "RL_Fz": RL_Fz_aligned_lst,
        #                "RR_Fx": RR_Fx_aligned_lst,
        #                "RR_Fy": RR_Fy_aligned_lst,
        #                "RR_Fz": RR_Fz_aligned_lst,
        #                }
            
        # df = pd.DataFrame(output_dict)

        # df.to_csv("./debugging.csv")

        # fig = plt.figure(figsize=plt.figaspect(2.))
        # fig.suptitle('YMD Debug Shit')

        # # y_ddot vs yaw_ddot vs x_ddot
        # ax = fig.add_subplot(4, 3, 1, projection='3d')

        # ax.scatter(y_ddot_lst, yaw_ddot_lst, x_ddot_lst)
        # ax.set_xlabel("y_ddot")
        # ax.set_ylabel("yaw_ddot")
        # ax.set_zlabel("x_ddot")

        # # YMD
        # ax = fig.add_subplot(4, 3, 2)
        # ax.scatter(y_ddot_lst, yaw_ddot_lst)
        # ax.set_xlabel("y_ddot")
        # ax.set_ylabel("yaw_ddot")

        # # FL alpha vs gamma vs Fy
        # ax = fig.add_subplot(4, 3, 3, projection='3d')

        # ax.scatter(FL_alpha_lst, FL_gamma_lst, FL_Fy_aligned_lst)
        # ax.set_xlabel("alpha")
        # ax.set_ylabel("gamma")
        # ax.set_zlabel("Fy")

        # # FL jounce vs Fz
        # ax = fig.add_subplot(4, 3, 4)

        # ax.scatter(FL_jounce_lst, FL_Fz_aligned_lst)
        # ax.scatter(FR_jounce_lst, FR_Fz_aligned_lst)
        # ax.scatter(RL_jounce_lst, RL_Fz_aligned_lst)
        # ax.scatter(RR_jounce_lst, RR_Fz_aligned_lst)
        # ax.legend(["FL", "FR", "RL", "RR"])
        # ax.set_xlabel("jounce")
        # ax.set_ylabel("Fz")

        # # FL jounce vs Fz
        # ax = fig.add_subplot(4, 3, 5, projection='3d')

        # ax.scatter(FL_alpha_lst, FL_Fz_aligned_lst, FL_Fy_aligned_lst)
        # ax.set_xlabel("alpha")
        # ax.set_ylabel("Fz")
        # ax.set_zlabel("Fy")

        # # Fz signals
        # ax = fig.add_subplot(4, 3, 6)

        # param = [x for x in range(len(FL_Fz_aligned_lst))]

        # ax.plot(param, FL_Fz_aligned_lst)
        # ax.plot(param, FR_Fz_aligned_lst)
        # ax.plot(param, RL_Fz_aligned_lst)
        # ax.plot(param, RR_Fz_aligned_lst)
        # ax.legend(["FL", "FR", "RL", "RR"])
        # ax.set_xlabel("Param")
        # ax.set_ylabel("Fz")

        # # alpha signals
        # ax = fig.add_subplot(4, 3, 7)

        # param = [x for x in range(len(FL_Fz_aligned_lst))]

        # ax.plot(param, FL_alpha_lst)
        # ax.plot(param, FR_alpha_lst)
        # ax.plot(param, RL_alpha_lst)
        # ax.plot(param, RR_alpha_lst)
        # ax.legend(["FL", "FR", "RL", "RR"])
        # ax.set_xlabel("Param")
        # ax.set_ylabel("alpha")

        # # Fy signals
        # ax = fig.add_subplot(4, 3, 8)

        # param = [x for x in range(len(FL_Fz_aligned_lst))]

        # ax.plot(param, FL_Fy_aligned_lst)
        # ax.plot(param, FR_Fy_aligned_lst)
        # ax.plot(param, RL_Fy_aligned_lst)
        # ax.plot(param, RR_Fy_aligned_lst)
        # ax.legend(["FL", "FR", "RL", "RR"])
        # ax.set_xlabel("Param")
        # ax.set_ylabel("Fy")

        # ax = fig.add_subplot(4, 3, 9)

        # param = [x for x in range(len(FL_Fz_aligned_lst))]

        # ax.scatter(FL_alpha_lst, delta_lst)
        # # ax.legend(["FL", "FR", "RL", "RR"])
        # ax.set_xlabel("alpha")
        # ax.set_ylabel("delta")

        # ax = fig.add_subplot(4, 3, 10)

        # param = [x for x in range(len(FL_Fz_aligned_lst))]

        # ax.plot(param, [x * 180 / np.pi for x in pitch_lst])
        # ax.plot(param, [x * 180 / np.pi for x in roll_lst])
        # ax.legend(["Pitch", "Roll"])
        # ax.set_xlabel("Param")
        # ax.set_ylabel("Angle (deg)")

        # ax.plot(t1, f(t1), 'bo',
        # t2, f(t2), 'k--', markerfacecolor='green')
        # ax.grid(True)
        # ax.set_ylabel('Damped oscillation')

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
        yaw_rate = 0 if ntb_accel[1] == 0 else ntb_accel[1] / np.linalg.norm(imf_velocity)

        # Store suspension objects
        suspension: SuspensionModel = self.suspension

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

        # Get CG position
        cgx = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgy = suspension.cgy_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgz = suspension.cgz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        
        self.cg_pos = np.array([cgx, cgy, cgz])

        # Store contact patch positions
        self.FL_Cp_wrt_cg = FL_double_wishbone.contact_patch.position - self.cg_pos
        self.FR_Cp_wrt_cg = FR_double_wishbone.contact_patch.position - self.cg_pos
        self.RL_Cp_wrt_cg = RL_double_wishbone.contact_patch.position - self.cg_pos
        self.RR_Cp_wrt_cg = RR_double_wishbone.contact_patch.position - self.cg_pos
        
        FL_jounce = suspension.FL_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_jounce = suspension.FR_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_jounce = suspension.RL_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_jounce = suspension.RR_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        sus_corners_jounce: Sequence[float] = [FL_jounce, FR_jounce, RL_jounce, RR_jounce]

        # Inclination angles
        FL_gamma = suspension.FL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_gamma = suspension.FR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_gamma = suspension.RL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_gamma = suspension.RR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # Toe angles
        FL_toe = suspension.FL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_toe = suspension.FR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_toe = suspension.RL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_toe = suspension.RR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

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

        for i in range(len(sus_corners)):
            if sus_corners_jounce[i] >= 0:
                Fz_lst.append(sus_corners[i].weight + sus_corners[i].wheelrate_function.integrate(0, sus_corners_jounce[i]))
            else:
                Fz_lst.append(sus_corners[i].weight - sus_corners[i].wheelrate_function.integrate(sus_corners_jounce[i], 0)) 

        FL_Fz, FR_Fz, RL_Fz, RR_Fz = Fz_lst

        tire_lift = any(list(filter(lambda x: x < 0, [FL_Fz, FR_Fz, RL_Fz, RR_Fz])))

        # Tire loads
        self.FL_forces = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[0:3]
        self.FR_forces = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[0:3]
        self.RL_forces = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[0:3]
        self.RR_forces = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[0:3]

        self.FL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_alpha), self.FL_forces)
        self.FR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_alpha), self.FR_forces)
        self.RL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_alpha), self.RL_forces)
        self.RR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_alpha), self.RR_forces)

        self.FL_moments = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[3:]
        self.FR_moments = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[3:]
        self.RL_moments = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[3:]
        self.RR_moments = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[3:]

        self.FL_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_alpha), self.FL_moments)
        self.FR_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_alpha), self.FR_moments)
        self.RL_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_alpha), self.RL_moments)
        self.RR_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_alpha), self.RR_moments)

        ###############################################
        ########### Aero Forces and Moments ###########
        ###############################################

        # aero_loads = self.vehicle.aero_model.force_props(roll=roll, pitch=pitch, body_slip=beta, heave=heave, velocity=velocity)

        # aero_forces = aero_loads[:3]
        # aero_FAP = aero_loads[3:]

        # CG_wrt_CoP = aero_FAP - self.cg_pos

        ###############################################
        ######## Calculate Forces and Moments #########
        ###############################################

        gravity_force = np.array([0, 0, self.vehicle.total_mass * self.vehicle.environment["G"]])

        suspension_forces = self.FL_forces_aligned + self.FR_forces_aligned + self.RL_forces_aligned + self.RR_forces_aligned
        suspension_moments = np.cross(self.FL_Cp_wrt_cg, self.FL_forces_aligned) + np.cross(self.FR_Cp_wrt_cg, self.FR_forces_aligned) + \
                             np.cross(self.RL_Cp_wrt_cg, self.RL_forces_aligned) + np.cross(self.RR_Cp_wrt_cg, self.RR_forces_aligned) + \
                             self.FL_moments_aligned + self.FR_moments_aligned + self.RL_moments_aligned + self.RR_moments_aligned
        
        aero_forces = 0
        aero_moments = 0

        vehicle_centric_forces = -1 * gravity_force + suspension_forces + aero_forces
        vehicle_centric_moments = suspension_moments + aero_moments

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

        residuals = np.array([*force_residuals, *moment_residuals])

        # if tire_lift:
            # lifted_tires = abs(np.array([x for x in Fz_lst if x < 0]))
            # lifted_tires = [x for x in Fz_lst if x < 0]
            # return residuals * sum(lifted_tires)

        return residuals

    def _residual_eval(self, x: Sequence[float], args: Sequence[float]) -> Sequence[float]:
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
        yaw_rate = 0 if ntb_accel[1] == 0 else ntb_accel[1] / np.linalg.norm(imf_velocity)

        # Store suspension objects
        suspension: SuspensionModel = self.suspension

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

        # Get CG position
        cgx = suspension.cgx_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgy = suspension.cgy_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        cgz = suspension.cgz_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        
        self.cg_pos = np.array([cgx, cgy, cgz])

        # Store contact patch positions
        self.FL_Cp_wrt_cg = FL_double_wishbone.contact_patch.position - self.cg_pos
        self.FR_Cp_wrt_cg = FR_double_wishbone.contact_patch.position - self.cg_pos
        self.RL_Cp_wrt_cg = RL_double_wishbone.contact_patch.position - self.cg_pos
        self.RR_Cp_wrt_cg = RR_double_wishbone.contact_patch.position - self.cg_pos
        
        FL_jounce = suspension.FL_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_jounce = suspension.FR_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_jounce = suspension.RL_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_jounce = suspension.RR_jounce_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        sus_corners_jounce: Sequence[float] = [FL_jounce, FR_jounce, RL_jounce, RR_jounce]

        # Inclination angles
        FL_gamma = suspension.FL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_gamma = suspension.FR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_gamma = suspension.RL_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_gamma = suspension.RR_gamma_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

        # Toe angles
        FL_toe = suspension.FL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        FR_toe = suspension.FR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RL_toe = suspension.RL_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]
        RR_toe = suspension.RR_toe_lookup(x=delta, y=heave, z=pitch, w=roll)[0]

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
        
        for i in range(len(sus_corners)):
            if sus_corners_jounce[i] >= 0:
                Fz_lst.append(sus_corners[i].weight + sus_corners[i].wheelrate_function.integrate(0, sus_corners_jounce[i]))
            else:
                Fz_lst.append(sus_corners[i].weight - sus_corners[i].wheelrate_function.integrate(sus_corners_jounce[i], 0)) 

        FL_Fz, FR_Fz, RL_Fz, RR_Fz = Fz_lst

        # Tire loads
        self.FL_forces = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[0:3]
        self.FR_forces = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[0:3]
        self.RL_forces = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[0:3]
        self.RR_forces = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[0:3]

        self.FL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_alpha), self.FL_forces)
        self.FR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_alpha), self.FR_forces)
        self.RL_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_alpha), self.RL_forces)
        self.RR_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_alpha), self.RR_forces)

        self.FL_moments = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)[3:]
        self.FR_moments = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)[3:]
        self.RL_moments = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)[3:]
        self.RR_moments = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)[3:]

        self.FL_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_alpha), self.FL_moments)
        self.FR_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_alpha), self.FR_moments)
        self.RL_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_alpha), self.RL_moments)
        self.RR_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_alpha), self.RR_moments)

        ###############################################
        ########### Aero Forces and Moments ###########
        ###############################################

        aero_loads = self.vehicle.aero_model.force_props(roll=roll, pitch=pitch, body_slip=beta, heave=heave, velocity=velocity)

        aero_forces = aero_loads[:3]
        aero_FAP = aero_loads[3:]

        CG_wrt_CoP = aero_FAP - self.cg_pos

        ###############################################
        ######## Calculate Forces and Moments #########
        ###############################################

        gravity_force = np.array([0, 0, self.vehicle.total_mass * self.vehicle.environment["G"]])

        suspension_forces = self.FL_forces_aligned + self.FR_forces_aligned + self.RL_forces_aligned + self.RR_forces_aligned
        suspension_moments = np.cross(self.FL_Cp_wrt_cg, self.FL_forces_aligned) + np.cross(self.FR_Cp_wrt_cg, self.FR_forces_aligned) + \
                             np.cross(self.RL_Cp_wrt_cg, self.RL_forces_aligned) + np.cross(self.RR_Cp_wrt_cg, self.RR_forces_aligned) 

        aero_forces = aero_forces
        aero_moments = np.cross(CG_wrt_CoP, aero_forces)

        vehicle_centric_forces = -1 * gravity_force + suspension_forces + aero_forces
        vehicle_centric_moments = suspension_moments + aero_moments

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

        residuals = [*force_residuals, *moment_residuals, FL_jounce, FR_jounce, RL_jounce, RR_jounce, FL_gamma, FR_gamma, RL_gamma, RR_gamma, FL_alpha, FR_alpha, RL_alpha, RR_alpha, *[float(x) for x in self.FL_forces_aligned], *[float(x) for x in self.FR_forces_aligned], *[float(x) for x in self.RL_forces_aligned], *[float(x) for x in self.RR_forces_aligned], heave, pitch, roll, delta, beta, ax, ay, yaw_ddot, self.FL_Cp_wrt_cg, self.FR_Cp_wrt_cg, self.RL_Cp_wrt_cg, self.RR_Cp_wrt_cg]

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
        ymd = plt.figure()
        ax = ymd.gca()

        ax.set_title("Yaw Acceleration vs Lateral Acceleration")
        ax.set_xlabel("Lateral Acceleration (m/s^2)")
        ax.set_ylabel("Yaw Acceleration (rad/s^2)")
        ax.axhline(c="gray", linewidth=0.5)
        ax.axvline(c="gray", linewidth=0.5)

        mp = self.mesh // 2

        for steered_angle, lat_accels, yaw_accels in self.steered_angle_iso_lines:
            a = 0.5 + np.sin(lat_accels[mp] ** 3 / 500) * 0.4
            text_pos = (lat_accels[mp] * a + lat_accels[mp + 1] * (1-a), yaw_accels[mp] * a + yaw_accels[mp + 1] * (1-a) + 0.2)

            ax.plot(lat_accels, yaw_accels, c="blue", linewidth=0.8)
            ax.scatter(lat_accels, yaw_accels, s=0.5, c="black")
            ax.text(text_pos[0], text_pos[1], f'δ = {round(steered_angle * 180 / np.pi, 1)}°', fontsize=6, c="blue")

        for body_slip, lat_accels, yaw_accels in self.body_slip_iso_lines:
            text_pos = (lat_accels[-1] + ((lat_accels[-1] < 0) * (lat_accels[-1] * 0.05 - 0.5)), yaw_accels[-1] + 0.7)

            ax.plot(lat_accels, yaw_accels, c="red", linewidth=0.8)
            ax.text(text_pos[0], text_pos[1], f'β = {round(body_slip * 180 / np.pi, 1)}°', fontsize=6, c="red")

        plt.show()