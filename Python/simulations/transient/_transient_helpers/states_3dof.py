from _4_custom_libraries.misc_math import rotation_matrix
from _4_custom_libraries.simulation import Simulation

from typing import Tuple, Sequence

import numpy as np


class States3DOF(Simulation):
    """## Transient Vehicle States: 3-DOF

    3-DOF transient physical model relating states to their derivatives and output equations

    State vector X (6 states):
    --------------------------
    1)  posX    : Longitudinal position (m)
    2)  posY    : Lateral position (m)
    3)  psi     : Yaw angle (rad)
    4)  velX    : Longitudinal velocity (m/s)
    5)  velY    : Lateral velocity (m/s)
    6)  velYaw  : Yaw velocity (rad/s)
    
    Output vector Y (11 outputs):
    -----------------------------
    1)   posX      : Longitudinal position (m)
    2)   posY      : Lateral position (m)
    3)   psi       : Yaw angle (rad)
    4)   velX      : Longitudinal velocity (m/s)
    5)   velY      : Lateral velocity (m/s)
    6)   yawVel    : Yaw velocity (rad/s)
    7)   accX_body : Longitudinal acceleration in body frame (m/s^2)
    8)   accY_body : Lateral acceleration in body frame (m/s^2)
    9)   accYaw    : Yaw acceleration (rad/s^2)
    10)  delta     : Steering angle (rad)
    11)  beta      : Sideslip angle (rad)

    Input vector U:
    ---------------
    1) handwheel_angle (rad)
    """
    def __init__(self, model_path: str) -> None:
        super().__init__(model_path=model_path)

        self.FL_alphas = []
        self.FL_Fys = []

    def der_state(self, x: Sequence[float], u: Tuple[float]) -> Sequence[float]:
        """## Der(State)

        Derivative of the state vector

        Parameters
        ----------
        x : Sequence[float]
            posX    : Longitudinal position (m)
            posY    : Lateral position (m)
            psi     : Yaw angle (rad)
            velX    : Longitudinal velocity (m/s)
            velY    : Lateral velocity (m/s)
            velYaw  : Yaw velocity (rad/s)

        u : Tuple[float]
            delta : front steering angle (rad)

        Returns
        -------
        Sequence[float]
            Derivative of the state vector
        """
        # Unpack
        posX, posY, psi, velX, velY, velYaw = x
        delta = u[0]
        
        # Dynamics (3DOF)
        accX, accY, accYaw = self._physical_model(velX, velY, velYaw, delta)
        
        # Kinematic integration for global motion
        dx_pos = velX * np.cos(psi) - velY * np.sin(psi)
        dy_pos = velX * np.sin(psi) + velY * np.cos(psi)
        dpsi = velYaw

        return [dx_pos, dy_pos, dpsi, accX, accY, accYaw]

    def _physical_model(self, velX: float, velY: float, velYaw: float, delta: float) -> Tuple[float, float, float]:
        """## Physical Model

        Physical model for 3DOF vehicle, computing 3-DOF accelerations

        Parameters
        ----------
        velX : float
            Longitudinal velocity (m/s)
        velY : float
            Lateral velocity (m/s)
        velYaw : float
            Yaw velocity (rad/s)
        delta : float
            Handwheel angle (rad)

        Returns
        -------
        Tuple[float, float, float]
            Accelerations of 3-DOF vehicle model
        """
        # === Unpack state and input ===
        hwa = delta

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

        # === Effective Contact Patch Positions ===
        Fr_cp = (np.array(FL_cp_pos) + np.array(FR_cp_pos)) / 2
        Rr_cp = (np.array(RL_cp_pos) + np.array(RR_cp_pos)) / 2

        # === Velocity at Each Effective Contact Patch ===
        Fr_cp_wrt_CG = Fr_cp - np.array(vehCG)
        Rr_cp_wrt_CG = Rr_cp - np.array(vehCG)

        Fr_vel = np.array([velX, velY, 0]) + np.cross([0, 0, velYaw], Fr_cp_wrt_CG)
        Rr_vel = np.array([velX, velY, 0]) + np.cross([0, 0, velYaw], Rr_cp_wrt_CG)

        # === Front and Rear Slip Angles ===
        FL_delta = self.kin_FMU["FL_delta"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        FR_delta = self.kin_FMU["FR_delta"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        RL_delta = self.kin_FMU["RL_delta"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        RR_delta = self.kin_FMU["RR_delta"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        
        Fr_delta = (FL_delta + FR_delta) / 2
        Rr_delta = (RL_delta + RR_delta) / 2

        Fr_alpha = Fr_delta - np.arctan2(Fr_vel[1], Fr_vel[0])
        Rr_alpha = Rr_delta - np.arctan2(Rr_vel[1], Rr_vel[0])

        # === Evaluate Tires ===
        FL_gamma = self.kin_FMU["FL_gamma"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        FR_gamma = self.kin_FMU["FR_gamma"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        RL_gamma = self.kin_FMU["RL_gamma"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180
        RR_gamma = self.kin_FMU["RR_gamma"](np.array([hwa * 180 / np.pi, 0, 0, 0]))[0] * np.pi / 180

        FL_tire_loads = FL_corner.tire.tire_eval(FZ=abs(FL_corner.static_weight), alpha=Fr_alpha, kappa=0, gamma=FL_gamma)
        FR_tire_loads = FR_corner.tire.tire_eval(FZ=abs(FR_corner.static_weight), alpha=Fr_alpha, kappa=0, gamma=FR_gamma)
        RL_tire_loads = RL_corner.tire.tire_eval(FZ=abs(RL_corner.static_weight), alpha=Rr_alpha, kappa=0, gamma=RL_gamma)
        RR_tire_loads = RR_corner.tire.tire_eval(FZ=abs(RR_corner.static_weight), alpha=Rr_alpha, kappa=0, gamma=RR_gamma)

        # === Decompose Loads ===
        Fr_tire_forces = np.array(FL_tire_loads[0:3]) + np.array(FR_tire_loads[0:3])
        Rr_tire_forces = np.array(RL_tire_loads[0:3]) + np.array(RR_tire_loads[0:3])

        Fr_tire_moments = np.array(FL_tire_loads[3:6]) + np.array(FR_tire_loads[3:6])
        Rr_tire_moments = np.array(RL_tire_loads[3:6]) + np.array(RR_tire_loads[3:6])

        # === Transform Into Vehicle Frame ===
        Fr_tire_forces_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(Fr_vel[1], Fr_vel[0])) @ Fr_tire_forces
        Rr_tire_forces_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(Rr_vel[1], Rr_vel[0])) @ Rr_tire_forces

        Fr_tire_moments_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(Fr_vel[1], Fr_vel[0])) @ Fr_tire_moments
        Rr_tire_moments_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(Rr_vel[1], Rr_vel[0])) @ Rr_tire_moments

        # === External Loads ===
        gravity_force = np.array([0, 0, self.sus.total_mass * self.sus_data.g])

        # === Sum Forces and Momments ===
        sum_F = Fr_tire_forces_aligned + Rr_tire_forces_aligned + gravity_force
        sum_M = np.cross(Fr_cp_wrt_CG, Fr_tire_forces_aligned) + \
                np.cross(Rr_cp_wrt_CG, Rr_tire_forces_aligned) + \
                Fr_tire_moments_aligned + \
                Rr_tire_moments_aligned

        accX = sum_F[0] / self.sus.total_mass
        accY = sum_F[1] / self.sus.total_mass
        accYaw = np.linalg.solve(self.sus.inertia_tensor, sum_M)[2]

        return [accX, accY, accYaw]

    def output_state(
              self,
              x: Sequence[float],
              der_x: Sequence[float],
              u: Tuple[float]
              ) -> Sequence[float]:
            """
            Compute output vector from current state, its derivative, and control input.

            Parameters
            ----------
            x : Tuple[float, float, float]
                posX    : Longitudinal position (m)
                posY    : Lateral position (m)
                psi     : Yaw angle (rad)
                velX    : Longitudinal velocity (m/s)
                velY    : Lateral velocity (m/s)
                velYaw  : Yaw velocity (rad/s)

            der_x : Tuple[float, float, float]
                dx_pos : d/dt (Longitudinal position) (m/s)
                dy_pos : d/dt (Lateral position) (m/s)
                dpsi   : d/dt (yaw angle) (rad/s)
                accX   : Longitudinal acceleration (m/s^2)
                accY   : Lateral acceleration (m/s^2)
                accYaw : Yaw acceleration (rad/s^2)

            u : Tuple[float]
                delta  : Front steering angle (rad)

            Returns
            -------
            y : Sequence[float]
                posX      : Longitudinal position (m)
                posY      : Lateral position (m)
                psi       : Yaw angle (rad)
                velX      : Longitudinal velocity (m/s)
                velY      : Lateral velocity (m/s)
                yawVel    : Yaw rate (rad/s)
                accX_body : Longitudinal acceleration in body frame (m/s^2)
                accY_body : Lateral acceleration in body frame (m/s^2)
                accYaw    : Yaw acceleration (rad/s^2)
                delta     : Steering angle (rad)
                beta      : Sideslip angle (rad)
                K_und     : Understeer gradient (rad / m/s^2)
            """
            posX, posY, psi, velX, velY, yawVel = x
            dx_pos, dy_pos, dpsi, accX, accY, accYaw = der_x
            delta = u[0]

            # Sideslip angle beta = atan(v_y / v_x)
            beta = np.arctan2(velY, velX) if velX != 0 else 0.0

            # Body-frame accelerations
            accX_body = accX - velY * yawVel
            accY_body = accY + velX * yawVel

            # Understeer gradient
            V = np.sqrt(velX**2 + velY**2)
            K_und = (delta - (self.sus.avg_wheelbase / V**2) * accY_body) / accY_body if abs(accY_body) > 0.05 else 0.0

            return [
                posX,
                posY,
                psi,
                velX,
                velY,
                yawVel,
                accX_body,
                accY_body,
                accYaw,
                delta,
                beta,
                K_und
            ]
