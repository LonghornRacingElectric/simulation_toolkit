from _4_custom_libraries.misc_math import rotation_matrix
from _4_custom_libraries.simulation import Simulation

from typing import Union, Tuple, Sequence
from scipy.optimize import fsolve

import numpy as np


class States6DOF(Simulation):
    """## Transient State Tracking: 6-DOF

    Transient physical model relating states to their derivatives and output equations

    State vector X (6 states):
    --------------------------
    1)  p_sprung_x  (linear momentum of sprung mass: x)
    2)  p_sprung_y  (linear momentum of sprung mass: y)
    3)  p_sprung_z  (linear momentum of sprung mass: z)
    4)  h_sprung_x  (angular momentum of sprung mass: roll)
    5)  h_sprung_y  (angular momentum of sprung mass: pitch)
    6)  h_sprung_z  (angular momentum of sprung mass: yaw)

    Internal states (6 states):
    -------------------------------
    1)      x_sprung  (linear displacement of sprung mass: x)
    2)      y_sprung  (linear displacement of sprung mass: y)
    3)      z_sprung  (linear displacement of sprung mass: z)
    4)    phi_sprung  (angular displacement of sprung mass: roll)
    5)  theta_sprung  (angular displacement of sprung mass: pitch)
    6)    psi_sprung  (angular displacement of sprung mass: yaw)

    Output vector Y (12 entries):
    -----------------------------
    1)     p_sprung_x  (linear momentum of sprung mass: x)
    2)     p_sprung_y  (linear momentum of sprung mass: y)
    3)     p_sprung_z  (linear momentum of sprung mass: z)
    4)     h_sprung_x  (angular momentum of sprung mass: roll)
    5)     h_sprung_y  (angular momentum of sprung mass: pitch)
    6)     h_sprung_z  (angular momentum of sprung mass: yaw)
    7)       x_sprung  (linear displacement of sprung mass: x)
    8)       y_sprung  (linear displacement of sprung mass: y)
    9)       z_sprung  (linear displacement of sprung mass: z)
    10)    phi_sprung  (angular displacement of sprung mass: roll)
    11)  theta_sprung  (angular displacement of sprung mass: pitch)
    12)    psi_sprung  (angular displacement of sprung mass: yaw)

    Input vector U:
    ---------------
    1) handwheel_angle (deg)
    """
    def __init__(self, model_path: str) -> None:
        super().__init__(model_path=model_path)

        self.FL_forces = []
        self.FR_forces = []
        self.RL_forces = []
        self.RR_forces = []

        self.FL_alphas = []
        self.FR_alphas = []
        self.RL_alphas = []
        self.RR_alphas = []

        self.FL_deltas = []
        self.FR_deltas = []
        self.RL_deltas = []
        self.RR_deltas = []

        self.FL_vels = []
        self.FR_vels = []
        self.RL_vels = []
        self.RR_vels = []

    def der_state_3dof(self, x: Sequence[float], u: Sequence[float]) -> Sequence[float]:
        """
        Linearized 3DOF bicycle model for transient vehicle response.

        Parameters
        ----------
        x : [vx, vy, r]
            vx : longitudinal velocity (m/s)
            vy : lateral velocity (m/s)
            r  : yaw rate (rad/s)

        u : [delta]
            delta : front steering angle (rad)

        Returns
        -------
        dx/dt : [ax, ay, yaw_acc]
        """
        # === Unpack state and input ===
        velX, velY, velYaw = x
        
        hwa = u[0]

        # === Relevant objects from sus
        FL_corner = self.sus.FL_quarter_car
        FR_corner = self.sus.FR_quarter_car
        RL_corner = self.sus.RL_quarter_car
        RR_corner = self.sus.RR_quarter_car

        FL_cp_pos = FL_corner.tire.contact_patch.initial_position
        FR_cp_pos = FR_corner.tire.contact_patch.initial_position
        RL_cp_pos = RL_corner.tire.contact_patch.initial_position
        RR_cp_pos = RR_corner.tire.contact_patch.initial_position

        vehCG = self.sus.CG_node.initial_position

        # === Model parameters ===
        m = self.sus.total_mass
        I_zz = self.sus.inertia_tensor[2][2]
        l_Fr = abs((FL_cp_pos[0] + FR_cp_pos[0]) / 2 - vehCG[0])
        l_Rr = abs((RL_cp_pos[0] + RR_cp_pos[0]) / 2 - vehCG[0])
        
        C_alpha_FL = FL_corner.tire.tire.get_cornering_stiffness(FZ=FL_corner.static_weight)
        C_alpha_FR = FR_corner.tire.tire.get_cornering_stiffness(FZ=FR_corner.static_weight)
        C_alpha_RL = RL_corner.tire.tire.get_cornering_stiffness(FZ=RL_corner.static_weight)
        C_alpha_RR = RR_corner.tire.tire.get_cornering_stiffness(FZ=RR_corner.static_weight)
        
        C_alpha_Fr = C_alpha_FL + C_alpha_FR
        C_alpha_Rr = C_alpha_RL + C_alpha_RR

        if abs(velX) < 1:
            raise Exception("velX fell below 1 m/s. Please initialize with a larger velocity.")

        # === Lateral tire forces (linear tire model) ===
        alpha_f = hwa - (velY + l_Fr * velYaw) / velX
        alpha_r = -(velY - l_Rr * velYaw) / velX

        Fy_Fr = C_alpha_Fr * alpha_f
        Fy_Rr = C_alpha_Rr * alpha_r

        # === Equations of motion ===
        accX = 0
        accY = (Fy_Fr + Fy_Rr) / m - velYaw * velX
        accYaw = (l_Fr * Fy_Fr - l_Rr * Fy_Rr) / I_zz

        return [accX, accY, accYaw]

    
    def der_state(self, x, u):
        # === Unpack state and control ===
        p = np.array(x[0:3])  # linear momentum
        h = np.array(x[3:6])  # angular momentum
        hwa = u[0]            # handwheel angle

        # === Compute velocity and angular velocity ===
        v = p / self.sus.total_mass
        omega = np.linalg.solve(self.sus.inertia_tensor, h)

        # === Get CG position and contact patch positions ===
        input_vec = np.array([hwa, 0, 0, 0])  # only hwa varies

        cg_pos = np.array([
            self.kin_FMU["veh_CG_x"](input_vec)[0],
            self.kin_FMU["veh_CG_y"](input_vec)[0],
            self.kin_FMU["veh_CG_z"](input_vec)[0]
        ])

        contact_patches = {}
        for corner in ["FL", "FR", "RL", "RR"]:
            cp_x = self.kin_FMU[f"{corner}_cp_x"](input_vec)[0]
            cp_y = self.kin_FMU[f"{corner}_cp_y"](input_vec)[0]
            cp_z = self.kin_FMU[f"{corner}_cp_z"](input_vec)[0]
            contact_patches[corner] = np.array([cp_x, cp_y, cp_z]) - cg_pos

        # === Tire kinematics (velocity, alpha, gamma, delta) ===
        tire_forces = {}
        tire_moments = {}

        for corner in ["FL", "FR", "RL", "RR"]:
            rel_pos = contact_patches[corner]
            vel = v + np.cross(omega, rel_pos)

            # Steering & camber angles
            gamma = np.deg2rad(self.kin_FMU[f"{corner}_gamma"](input_vec)[0])
            delta = np.deg2rad(self.kin_FMU[f"{corner}_delta"](input_vec)[0])

            # Slip angle
            alpha = delta - np.arctan2(vel[1], vel[0])

            # Vertical load (approx static, or from separate solver)
            Fz = getattr(self.sus, f"{corner}_quarter_car").static_weight

            # === Tire force in tire frame ===
            tire_out = getattr(self.sus_data, f"{corner}_quarter_car").tire.tire_eval(FZ=Fz, alpha=alpha, kappa=0, gamma=gamma)
            force_tire = tire_out[0:3]
            moment_tire = tire_out[3:6]

            # === Rotate force/moment from tire frame to vehicle frame ===
            # Assume tire frame is rotated about Z by delta
            R = rotation_matrix([0, 0, 1], theta=delta)
            force_vehicle = np.array(R) @ np.array(force_tire)
            moment_vehicle = np.array(R) @ np.array(moment_tire)

            tire_forces[corner] = force_vehicle
            tire_moments[corner] = moment_vehicle

        # === Sum forces and moments ===
        gravity_force = np.array([0, 0, -9.81 * self.sus.total_mass])
        total_force = gravity_force + sum(tire_forces.values())

        total_moment = np.zeros(3)
        for corner in ["FL", "FR", "RL", "RR"]:
            r = contact_patches[corner]
            F = tire_forces[corner]
            M = tire_moments[corner]
            total_moment += np.cross(r, F) + M

        # === Output derivative of state ===
        return [*total_force, *total_moment]

    # def der_state(self, x: Sequence[float], u: Sequence[float]) -> Sequence[float]:
    #     """## Derivative of State Vector
        
    #     Derivative of the given state vector

    #     Parameters
    #     ----------
    #     x : Sequence[float]
    #         Initial condition state vector
    #     der_x : Sequence[float]
    #         Initial condition der_state vector
    #     z : Sequence[float]
    #         Initial condition internal states
    #     u : Sequence[float]
    #         Control input vector
        
    #     Returns
    #     -------
    #     Sequence[float]
    #         Derivative of the state vector
    #     """

    #     # val = self.sus.FL_quarter_car.tire.tire_eval(FZ=1000, alpha=5 * np.pi / 180, kappa=0, gamma=0 * np.pi / 180)
    #     # print(val)
    #     # raise Exception
        
    #     # Control input
    #     hwa      = u[0]

    #     # State vector
    #     p_sprung = x[0:3]
    #     h_sprung = x[3:6]

    #     # Extract information from previous state and derivative
    #     vehVel = np.array(p_sprung) / self.sus.total_mass
    #     omega_vec = np.linalg.solve(self.sus.inertia_tensor, h_sprung)

    #     if np.linalg.norm(vehVel) < 1e-12:
    #         beta = 0.0
    #     else:
    #         beta = np.arctan(vehVel[1] / vehVel[0])

    #     # Get contact patch locations
    #     # FL_cp_x = self.kin_FMU["FL_cp_x"](np.array([hwa, 0, 0, 0]))[0]
    #     # FL_cp_y = self.kin_FMU["FL_cp_y"](np.array([hwa, 0, 0, 0]))[0]
    #     # FL_cp_z = self.kin_FMU["FL_cp_z"](np.array([hwa, 0, 0, 0]))[0]

    #     # FR_cp_x = self.kin_FMU["FR_cp_x"](np.array([hwa, 0, 0, 0]))[0]
    #     # FR_cp_y = self.kin_FMU["FR_cp_y"](np.array([hwa, 0, 0, 0]))[0]
    #     # FR_cp_z = self.kin_FMU["FR_cp_z"](np.array([hwa, 0, 0, 0]))[0]

    #     # RL_cp_x = self.kin_FMU["RL_cp_x"](np.array([hwa, 0, 0, 0]))[0]
    #     # RL_cp_y = self.kin_FMU["RL_cp_y"](np.array([hwa, 0, 0, 0]))[0]
    #     # RL_cp_z = self.kin_FMU["RL_cp_z"](np.array([hwa, 0, 0, 0]))[0]

    #     # RR_cp_x = self.kin_FMU["RR_cp_x"](np.array([hwa, 0, 0, 0]))[0]
    #     # RR_cp_y = self.kin_FMU["RR_cp_y"](np.array([hwa, 0, 0, 0]))[0]
    #     # RR_cp_z = self.kin_FMU["RR_cp_z"](np.array([hwa, 0, 0, 0]))[0]

    #     # FL_cp_pos = np.array([FL_cp_x, FL_cp_y, FL_cp_z])
    #     # FR_cp_pos = np.array([FR_cp_x, FR_cp_y, FR_cp_z])
    #     # RL_cp_pos = np.array([RL_cp_x, RL_cp_y, RL_cp_z])
    #     # RR_cp_pos = np.array([RR_cp_x, RR_cp_y, RR_cp_z])

    #     FL_cp_pos = np.array(self.sus.FL_quarter_car.tire.contact_patch.initial_position)
    #     FR_cp_pos = np.array(self.sus.FR_quarter_car.tire.contact_patch.initial_position)
    #     RL_cp_pos = np.array(self.sus.RL_quarter_car.tire.contact_patch.initial_position)
    #     RR_cp_pos = np.array(self.sus.RR_quarter_car.tire.contact_patch.initial_position)

    #     # Get CG position
    #     # cg_x = self.kin_FMU["veh_CG_x"](np.array([hwa, 0, 0, 0]))[0]
    #     # cg_y = self.kin_FMU["veh_CG_y"](np.array([hwa, 0, 0, 0]))[0]
    #     # cg_z = self.kin_FMU["veh_CG_z"](np.array([hwa, 0, 0, 0]))[0]

    #     # cg_pos = np.array([cg_x, cg_y, cg_z])

    #     cg_pos = self.sus.CG_node.initial_position

    #     # Get general geometry
    #     Fr_track = abs(FL_cp_pos[1] - FR_cp_pos[1])
    #     Rr_track = abs(RL_cp_pos[1] - RR_cp_pos[1])
    #     wheelbase = abs(FL_cp_pos[0] - RL_cp_pos[0])

    #     FL_Cp_wrt_cg = np.array(FL_cp_pos) - np.array(cg_pos)
    #     FR_Cp_wrt_cg = np.array(FR_cp_pos) - np.array(cg_pos)
    #     RL_Cp_wrt_cg = np.array(RL_cp_pos) - np.array(cg_pos)
    #     RR_Cp_wrt_cg = np.array(RR_cp_pos) - np.array(cg_pos)

    #     # Wheel translational velocities
    #     FL_velocity = vehVel + np.cross(omega_vec, FL_Cp_wrt_cg)
    #     FR_velocity = vehVel + np.cross(omega_vec, FR_Cp_wrt_cg)
    #     RL_velocity = vehVel + np.cross(omega_vec, RL_Cp_wrt_cg)
    #     RR_velocity = vehVel + np.cross(omega_vec, RR_Cp_wrt_cg)

    #     # Wheel inclination angles
    #     FL_gamma = self.kin_FMU["FL_gamma"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180
    #     FR_gamma = self.kin_FMU["FR_gamma"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180
    #     RL_gamma = self.kin_FMU["RL_gamma"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180
    #     RR_gamma = self.kin_FMU["RR_gamma"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180

    #     # Wheel delta angles
    #     # FL_delta = self.kin_FMU["FL_delta"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180
    #     # FR_delta = self.kin_FMU["FR_delta"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180
    #     # RL_delta = self.kin_FMU["RL_delta"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180
    #     # RR_delta = self.kin_FMU["RR_delta"](np.array([hwa, 0, 0, 0]))[0] * np.pi / 180

    #     FL_delta = hwa / 30 * 7 * np.pi / 180
    #     FR_delta = hwa / 30 * 7 * np.pi / 180
    #     RL_delta = 0 * np.pi / 180
    #     RR_delta = 0 * np.pi / 180
        
    #     self.FL_deltas.append(FL_delta * 180 / np.pi)
    #     self.FR_deltas.append(FR_delta * 180 / np.pi)
    #     self.RL_deltas.append(RL_delta * 180 / np.pi)
    #     self.RR_deltas.append(RR_delta * 180 / np.pi)

    #     self.FL_vels.append(np.arctan2(FL_velocity[1], FL_velocity[0]) * 180 / np.pi)
    #     self.FR_vels.append(np.arctan2(FR_velocity[1], FR_velocity[0]) * 180 / np.pi)
    #     self.RL_vels.append(np.arctan2(RL_velocity[1], RL_velocity[0]) * 180 / np.pi)
    #     self.RR_vels.append(np.arctan2(RR_velocity[1], RR_velocity[0]) * 180 / np.pi)

    #     FL_alpha = FL_delta - np.arctan2(FL_velocity[1], FL_velocity[0])
    #     FR_alpha = FR_delta - np.arctan2(FR_velocity[1], FR_velocity[0])
    #     RL_alpha = RL_delta - np.arctan2(RL_velocity[1], RL_velocity[0])
    #     RR_alpha = RR_delta - np.arctan2(RR_velocity[1], RR_velocity[0])

    #     self.FL_alphas.append(FL_alpha * 180 / np.pi)
    #     self.FR_alphas.append(FR_alpha * 180 / np.pi)
    #     self.RL_alphas.append(RL_alpha * 180 / np.pi)
    #     self.RR_alphas.append(RR_alpha * 180 / np.pi)

    #     # Fr_LLTD = self.sus.Fr_Kr / (self.sus.Fr_Kr + self.sus.Rr_Kr)

    #     # def constraint(x):
    #     #     accX = x[0]
    #     #     accY = x[1]
    #     #     accYaw = x[2]
    #     #     FL_Fz_guess = x[3]
    #     #     FR_Fz_guess = x[4]
    #     #     RL_Fz_guess = x[5]
    #     #     RR_Fz_guess = x[6]
            
    #     #     lat_LT = self.sus.total_mass * accY * cg_pos[2] / ((Fr_track + Rr_track) / 2)
    #     #     long_LT = self.sus.total_mass * accX * cg_pos[2] / wheelbase

    #     #     FL_Fz = self.sus.FL_quarter_car.static_weight - lat_LT / 2 - long_LT / 2
    #     #     FR_Fz = self.sus.FR_quarter_car.static_weight + lat_LT / 2 - long_LT / 2
    #     #     RL_Fz = self.sus.RL_quarter_car.static_weight - lat_LT / 2 + long_LT / 2
    #     #     RR_Fz = self.sus.RR_quarter_car.static_weight + lat_LT / 2 + long_LT / 2

    #     #     if FL_Fz < 0:
    #     #         FL_tire_output = np.zeros(6)
    #     #     else:
    #     #         FL_tire_output = self.sus_data.FL_quarter_car.tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma * 0)
    #     #     if FR_Fz < 0:
    #     #         FR_tire_output = np.zeros(6)
    #     #     else:
    #     #         FR_tire_output = self.sus_data.FR_quarter_car.tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma * 0)
    #     #     if RL_Fz < 0:
    #     #         RL_tire_output = np.zeros(6)
    #     #     else:
    #     #         RL_tire_output = self.sus_data.RL_quarter_car.tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma * 0)
    #     #     if RR_Fz < 0:
    #     #         RR_tire_output = np.zeros(6)
    #     #     else:
    #     #         RR_tire_output = self.sus_data.RR_quarter_car.tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma * 0)

    #     #     FL_tire_forces = FL_tire_output[0:3]
    #     #     FR_tire_forces = FR_tire_output[0:3]
    #     #     RL_tire_forces = RL_tire_output[0:3]
    #     #     RR_tire_forces = RR_tire_output[0:3]

    #     #     FL_tire_moments = FL_tire_output[3:6]
    #     #     FR_tire_moments = FR_tire_output[3:6]
    #     #     RL_tire_moments = RL_tire_output[3:6]
    #     #     RR_tire_moments = RR_tire_output[3:6]

    #     #     FL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FL_velocity[1], FL_velocity[0])), FL_tire_forces)
    #     #     FR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FR_velocity[1], FR_velocity[0])), FR_tire_forces)
    #     #     RL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RL_velocity[1], RL_velocity[0])), RL_tire_forces)
    #     #     RR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RR_velocity[1], RR_velocity[0])), RR_tire_forces)

    #     #     FL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FL_velocity[1], FL_velocity[0])), FL_tire_moments)
    #     #     FR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FR_velocity[1], FR_velocity[0])), FR_tire_moments)
    #     #     RL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RL_velocity[1], RL_velocity[0])), RL_tire_moments)
    #     #     RR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RR_velocity[1], RR_velocity[0])), RR_tire_moments)
            
    #     #     # Sum forces and moments yippee
            
    #     #     gravity_force = np.array([0, 0, self.sus.total_mass * -9.81])

    #     #     suspension_forces = FL_tire_forces_aligned + \
    #     #                         FR_tire_forces_aligned + \
    #     #                         RL_tire_forces_aligned + \
    #     #                         RR_tire_forces_aligned
    #     #     suspension_moments = np.cross(FL_Cp_wrt_cg, FL_tire_forces_aligned) + \
    #     #                          np.cross(FR_Cp_wrt_cg, FR_tire_forces_aligned) + \
    #     #                          np.cross(RL_Cp_wrt_cg, RL_tire_forces_aligned) + \
    #     #                          np.cross(RR_Cp_wrt_cg, RR_tire_forces_aligned) + \
    #     #                          FL_tire_moments_aligned + \
    #     #                          FR_tire_moments_aligned + \
    #     #                          RL_tire_moments_aligned + \
    #     #                          RR_tire_moments_aligned

    #     #     sum_forces = np.array(suspension_forces) + gravity_force
    #     #     sum_moments = suspension_moments
            
    #     #     accX_calc = sum_forces[0] / self.sus.total_mass
    #     #     accY_calc = sum_forces[1] / self.sus.total_mass
    #     #     accYaw_calc = np.linalg.solve(self.sus.inertia_tensor, sum_moments)[2]

    #     #     return [accX - accX_calc, accY - accY_calc, accYaw - accYaw_calc, FL_Fz_guess - FL_Fz, FR_Fz_guess - FR_Fz, RL_Fz_guess - RL_Fz, RR_Fz_guess - RR_Fz]

    #     # x0 = [0.1, 
    #     #       0.1,
    #     #       0.1, 
    #     #       self.sus.FL_quarter_car.static_weight, 
    #     #       self.sus.FR_quarter_car.static_weight, 
    #     #       self.sus.RL_quarter_car.static_weight, 
    #     #       self.sus.RR_quarter_car.static_weight]
        
    #     # _, _, _, FL_Fz, FR_Fz, RL_Fz, RR_Fz = fsolve(func=constraint, x0=x0)

    #     FL_Fz = self.sus.FL_quarter_car.static_weight
    #     FR_Fz = self.sus.FR_quarter_car.static_weight
    #     RL_Fz = self.sus.RL_quarter_car.static_weight
    #     RR_Fz = self.sus.RR_quarter_car.static_weight

    #     if FL_Fz < 0:
    #         FL_tire_output = np.zeros(6)
    #     else:
    #         FL_tire_output = self.sus_data.FL_quarter_car.tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)
    #     if FR_Fz < 0:
    #         FR_tire_output = np.zeros(6)
    #     else:
    #         FR_tire_output = self.sus_data.FR_quarter_car.tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)
    #     if RL_Fz < 0:
    #         RL_tire_output = np.zeros(6)
    #     else:
    #         RL_tire_output = self.sus_data.RL_quarter_car.tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)
    #     if RR_Fz < 0:
    #         RR_tire_output = np.zeros(6)
    #     else:
    #         RR_tire_output = self.sus_data.RR_quarter_car.tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)

    #     FL_tire_forces = FL_tire_output[0:3]
    #     FR_tire_forces = FR_tire_output[0:3]
    #     RL_tire_forces = RL_tire_output[0:3]
    #     RR_tire_forces = RR_tire_output[0:3]

    #     FL_tire_moments = FL_tire_output[3:6]
    #     FR_tire_moments = FR_tire_output[3:6]
    #     RL_tire_moments = RL_tire_output[3:6]
    #     RR_tire_moments = RR_tire_output[3:6]

    #     FL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FL_velocity[1], FL_velocity[0])), FL_tire_forces)
    #     FR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FR_velocity[1], FR_velocity[0])), FR_tire_forces)
    #     RL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RL_velocity[1], RL_velocity[0])), RL_tire_forces)
    #     RR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RR_velocity[1], RR_velocity[0])), RR_tire_forces)

    #     FL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FL_velocity[1], FL_velocity[0])), FL_tire_moments)
    #     FR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FR_velocity[1], FR_velocity[0])), FR_tire_moments)
    #     RL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RL_velocity[1], RL_velocity[0])), RL_tire_moments)
    #     RR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RR_velocity[1], RR_velocity[0])), RR_tire_moments)

    #     self.FL_forces.append(FL_tire_forces)
    #     self.FR_forces.append(FR_tire_forces)
    #     self.RL_forces.append(RL_tire_forces)
    #     self.RR_forces.append(RR_tire_forces)

    #     ###############################################
    #     ########### Aero Forces and Moments ###########
    #     ###############################################

    #     # aero = Aero("./_1_model_inputs/aero_map.csv")

    #     # aero_loads = aero.eval(roll=0, pitch=0, yaw=beta * 180 / np.pi, vel=np.linalg.norm(vehVel))

    #     ###############################################
    #     ######## Calculate Forces and Moments #########
    #     ###############################################

    #     gravity_force = np.array([0, 0, self.sus.total_mass * -9.81])

    #     suspension_forces = FL_tire_forces_aligned + \
    #                         FR_tire_forces_aligned + \
    #                         RL_tire_forces_aligned + \
    #                         RR_tire_forces_aligned
    #     suspension_moments = np.cross(FL_Cp_wrt_cg, FL_tire_forces_aligned) + \
    #                          np.cross(FR_Cp_wrt_cg, FR_tire_forces_aligned) + \
    #                          np.cross(RL_Cp_wrt_cg, RL_tire_forces_aligned) + \
    #                          np.cross(RR_Cp_wrt_cg, RR_tire_forces_aligned) # + \
    #                         #  FL_tire_moments_aligned + \
    #                         #  FR_tire_moments_aligned + \
    #                         #  RL_tire_moments_aligned + \
    #                         #  RR_tire_moments_aligned

    #     # aero_forces = aero_loads[:3]
    #     # aero_moments = aero_loads[3:]

    #     aero_forces = np.zeros(3)
    #     aero_moments = np.zeros(3)

    #     vehicle_centric_forces = np.array(suspension_forces) + gravity_force # + np.array(aero_forces)
    #     vehicle_centric_moments = suspension_moments # + aero_moments

    #     # The derivative of momentum is load, so we're done here
    #     return [*vehicle_centric_forces, *vehicle_centric_moments]