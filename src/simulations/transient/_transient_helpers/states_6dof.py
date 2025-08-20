from _4_custom_libraries.misc_math import rotation_matrix
from _4_custom_libraries.simulation import Simulation

from typing import Tuple, Sequence

import numpy as np


class States6DOF(Simulation):
    """## Transient Vehicle States: 6-DOF

    6-DOF transient physical model relating states to their derivatives and output equations

    State vector X (12 states):
    --------------------------
    All states apply specifically to the SPRUNG MASS:
    1)   posX     : Longitudinal position (m)
    2)   posY     : Lateral position (m)
    3)   posZ     : Vertical position (m)
    4)   phi      : Roll angle (rad)
    5)   theta    : Pitch angle (rad)
    6)   psi      : Yaw angle (rad)
    7)   velX     : Longitudinal velocity (m/s)
    8)   velY     : Lateral velocity (m/s)
    9)   velZ     : Vertical velocity (m/s)
    10)  vehRoll  : Roll velocity (rad/s)
    11)  vehPitch : Pitch velocity (rad/s)
    12)  vehYaw   : Yaw velocity (rad/s)

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
            posX     : Longitudinal position (m)
            posY     : Lateral position (m)
            posZ     : Vertical position (m)
            phi      : Roll angle (rad)
            theta    : Pitch angle (rad)
            psi      : Yaw angle (rad)
            velX     : Longitudinal velocity (m/s)
            velY     : Lateral velocity (m/s)
            velZ     : Vertical velocity (m/s)
            velRoll  : Roll velocity (rad/s)
            velPitch : Pitch velocity (rad/s)
            velYaw   : Yaw velocity (rad/s)

        u : Tuple[float]
            delta : front steering angle (rad)

        Returns
        -------
        Sequence[float]
            Derivative of the state vector
        """
        # Unpack
        posX, posY, posZ = x[0:3]
        phi, theta, psi = x[3:6]
        velX, velY, velZ = x[6:9]
        velRoll, velPitch, velYaw = x[9:12]
        
        delta = u[0]

        # Get rotation matrices for each rotation axis
        R_roll = np.array(rotation_matrix(  [1, 0, 0],   phi))
        R_pitch = np.array(rotation_matrix( [0, 1, 0], theta))
        R_yaw = np.array(rotation_matrix(   [0, 0, 1],   psi))

        R_body_to_inertial = R_yaw @ R_pitch @ R_roll

        # Transform body-frame linear velocity to inertial frame (global frame)
        vel_body = np.array([velX, velY, velZ])
        vel_inertial = R_body_to_inertial @ vel_body

        dx_pos, dy_pos, dz_pos = vel_inertial

        # Obtain derivatives from 6DOF model
        der_physical_model = self._physical_model(delta, posZ, theta, phi, velX, velY, velZ, velRoll, velPitch, velYaw)
        
        # Extract derivatives
        accX, accY, accZ, accRoll, accPitch, accYaw = der_physical_model

        # Angular velocities are rates of change of angles
        dphi = velRoll
        dtheta = velPitch
        dpsi = velYaw

        # Return derivative of state vector
        return [dx_pos, dy_pos, dz_pos, dphi, dtheta, dpsi, accX, accY, accZ, accRoll, accPitch, accYaw]
    
    def _physical_model(self, 
                        delta:    float,
                        posZ:     float,
                        theta:    float,
                        phi:      float,
                        velX:     float,
                        velY:     float,
                        velZ:     float,
                        velRoll:  float,
                        velPitch: float,
                        velYaw:   float) -> Sequence[float]:
        """## Physical Model

        Physical model for 6DOF vehicle, computing 6-DOF accelerations

        Parameters
        ----------
        delta : float
            Handwheel angle (rad)
        posZ : float
            Vertical position of sprung mass (m)
        theta : float
            Pitch of sprung mass (rad)
        phi : float
            Roll of sprung mass (rad)
        velX : float
            Longitudinal velocity (m/s)
        velY : float
            Lateral velocity (m/s)
        velZ : float
            Vertical velocity of sprung mass (rad/s)
        velRoll : float
            Roll velocity of sprung mass (rad/s)
        velPitch : float
            Pitch velocity of sprung mass (rad/s)
        velYaw : float
            Yaw velocity of sprung mass (rad/s)

        Returns
        -------
        Sequence[float]
            Accelerations of 6-DOF vehicle model
        """
        # === Unpack state and input ===
        hwa = delta
        heave, pitch, roll = 1.5 * 0.0254 - posZ, theta * 180 / np.pi, phi * 180 / np.pi
        # velX, velY, velZ, velRoll, velPitch, velYaw

        # === Relevant objects from sus
        FL_corner = self.sus.FL_quarter_car
        FR_corner = self.sus.FR_quarter_car
        RL_corner = self.sus.RL_quarter_car
        RR_corner = self.sus.RR_quarter_car

        # === Get contact patch locations ===
        FL_cp_x = self.kin_FMU["FL_cp_x"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FL_cp_y = self.kin_FMU["FL_cp_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FL_cp_z = self.kin_FMU["FL_cp_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        FR_cp_x = self.kin_FMU["FR_cp_x"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FR_cp_y = self.kin_FMU["FR_cp_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FR_cp_z = self.kin_FMU["FR_cp_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        RL_cp_x = self.kin_FMU["RL_cp_x"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RL_cp_y = self.kin_FMU["RL_cp_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RL_cp_z = self.kin_FMU["RL_cp_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        RR_cp_x = self.kin_FMU["RR_cp_x"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RR_cp_y = self.kin_FMU["RR_cp_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RR_cp_z = self.kin_FMU["RR_cp_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        FL_cp_pos = np.array([FL_cp_x, FL_cp_y, FL_cp_z])
        FR_cp_pos = np.array([FR_cp_x, FR_cp_y, FR_cp_z])
        RL_cp_pos = np.array([RL_cp_x, RL_cp_y, RL_cp_z])
        RR_cp_pos = np.array([RR_cp_x, RR_cp_y, RR_cp_z])

        # === Get CG Location ===
        cg_x = self.kin_FMU["veh_CG_x"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        cg_y = self.kin_FMU["veh_CG_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        cg_z = self.kin_FMU["veh_CG_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        vehCG = np.array([cg_x, cg_y, cg_z])

        # === Velocity at Each Contact Patch ===
        FL_cp_wrt_CG = FL_cp_pos - vehCG
        FR_cp_wrt_CG = FR_cp_pos - vehCG
        RL_cp_wrt_CG = RL_cp_pos - vehCG
        RR_cp_wrt_CG = RR_cp_pos - vehCG

        FL_vel = np.array([velX, velY, velZ]) + np.cross([velRoll, velPitch, velYaw], FL_cp_wrt_CG)
        FR_vel = np.array([velX, velY, velZ]) + np.cross([velRoll, velPitch, velYaw], FR_cp_wrt_CG)
        RL_vel = np.array([velX, velY, velZ]) + np.cross([velRoll, velPitch, velYaw], RL_cp_wrt_CG)
        RR_vel = np.array([velX, velY, velZ]) + np.cross([velRoll, velPitch, velYaw], RR_cp_wrt_CG)
        
        # === Slip Angles ===
        FL_delta = self.kin_FMU["FL_delta"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        FR_delta = self.kin_FMU["FR_delta"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        RL_delta = self.kin_FMU["RL_delta"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        RR_delta = self.kin_FMU["RR_delta"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        
        FL_alpha = FL_delta - np.arctan2(FL_vel[1], FL_vel[0])
        FR_alpha = FR_delta - np.arctan2(FR_vel[1], FR_vel[0])
        RL_alpha = RL_delta - np.arctan2(RL_vel[1], RL_vel[0])
        RR_alpha = RR_delta - np.arctan2(RR_vel[1], RR_vel[0])

        # if str(FL_alpha) == str(np.nan):
        #      raise Exception
        # else:
        #     print(FL_delta * 180 / np.pi)
        
        # === Evaluate Tires ===
        FL_gamma = self.kin_FMU["FL_gamma"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        FR_gamma = self.kin_FMU["FR_gamma"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        RL_gamma = self.kin_FMU["RL_gamma"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180
        RR_gamma = self.kin_FMU["RR_gamma"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0] * np.pi / 180

        FL_tire_loads = FL_corner.tire.tire_eval(FZ=abs(FL_corner.static_weight), alpha=FL_alpha, kappa=0, gamma=FL_gamma)
        FR_tire_loads = FR_corner.tire.tire_eval(FZ=abs(FR_corner.static_weight), alpha=FR_alpha, kappa=0, gamma=FR_gamma)
        RL_tire_loads = RL_corner.tire.tire_eval(FZ=abs(RL_corner.static_weight), alpha=RL_alpha, kappa=0, gamma=RL_gamma)
        RR_tire_loads = RR_corner.tire.tire_eval(FZ=abs(RR_corner.static_weight), alpha=RR_alpha, kappa=0, gamma=RR_gamma)

        # === Decompose Loads ===
        FL_tire_forces = np.array(FL_tire_loads[0:3])
        FR_tire_forces = np.array(FR_tire_loads[0:3])
        RL_tire_forces = np.array(RL_tire_loads[0:3])
        RR_tire_forces = np.array(RR_tire_loads[0:3])

        FL_tire_moments = np.array(FL_tire_loads[3:6])
        FR_tire_moments = np.array(FR_tire_loads[3:6])
        RL_tire_moments = np.array(RL_tire_loads[3:6])
        RR_tire_moments = np.array(RR_tire_loads[3:6])

        # === Transform Into Vehicle Frame ===
        FL_tire_forces_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FL_vel[1], FL_vel[0])) @ FL_tire_forces
        FR_tire_forces_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FR_vel[1], FR_vel[0])) @ FR_tire_forces
        RL_tire_forces_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RL_vel[1], RL_vel[0])) @ RL_tire_forces
        RR_tire_forces_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RR_vel[1], RR_vel[0])) @ RR_tire_forces

        FL_tire_moments_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FL_vel[1], FL_vel[0])) @ FL_tire_moments
        FR_tire_moments_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(FR_vel[1], FR_vel[0])) @ FR_tire_moments
        RL_tire_moments_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RL_vel[1], RL_vel[0])) @ RL_tire_moments
        RR_tire_moments_aligned = rotation_matrix(unit_vec=[0, 0, 1], theta=np.arctan2(RR_vel[1], RR_vel[0])) @ RR_tire_moments

        # === Compute Instant Links ===
        FL_FVIC_y = self.kin_FMU["FL_FVIC_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FL_FVIC_z = self.kin_FMU["FL_FVIC_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        
        FR_FVIC_y = self.kin_FMU["FR_FVIC_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FR_FVIC_z = self.kin_FMU["FR_FVIC_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        RL_FVIC_y = self.kin_FMU["RL_FVIC_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RL_FVIC_z = self.kin_FMU["RL_FVIC_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        RR_FVIC_y = self.kin_FMU["RR_FVIC_y"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RR_FVIC_z = self.kin_FMU["RR_FVIC_z"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        
        FL_FVIC_pos = np.array([FL_cp_pos[0], FL_FVIC_y, FL_FVIC_z])
        FR_FVIC_pos = np.array([FR_cp_pos[0], FR_FVIC_y, FR_FVIC_z])
        RL_FVIC_pos = np.array([RL_cp_pos[0], RL_FVIC_y, RL_FVIC_z])
        RR_FVIC_pos = np.array([RR_cp_pos[0], RR_FVIC_y, RR_FVIC_z])

        FL_FVIC_link = FL_FVIC_pos - FL_cp_pos
        FR_FVIC_link = FR_FVIC_pos - FR_cp_pos
        RL_FVIC_link = RL_FVIC_pos - RL_cp_pos
        RR_FVIC_link = RR_FVIC_pos - RR_cp_pos

        # === Project Tire Forces ===
        FL_FVIC_e_link = FL_FVIC_link / np.linalg.norm(FL_FVIC_link)
        FR_FVIC_e_link = FR_FVIC_link / np.linalg.norm(FR_FVIC_link)
        RL_FVIC_e_link = RL_FVIC_link / np.linalg.norm(RL_FVIC_link)
        RR_FVIC_e_link = RR_FVIC_link / np.linalg.norm(RR_FVIC_link)

        FL_tire_forces_projected = np.dot(FL_tire_forces_aligned, FL_FVIC_e_link) * FL_FVIC_e_link
        FR_tire_forces_projected = np.dot(FR_tire_forces_aligned, FR_FVIC_e_link) * FR_FVIC_e_link
        RL_tire_forces_projected = np.dot(RL_tire_forces_aligned, RL_FVIC_e_link) * RL_FVIC_e_link
        RR_tire_forces_projected = np.dot(RR_tire_forces_aligned, RR_FVIC_e_link) * RR_FVIC_e_link

        # === Compute Spring Forces ===
        FL_Fz_resid = (FL_tire_forces_aligned - FL_tire_forces_projected)[2]
        FR_Fz_resid = (FR_tire_forces_aligned - FR_tire_forces_projected)[2]
        RL_Fz_resid = (RL_tire_forces_aligned - RL_tire_forces_projected)[2]
        RR_Fz_resid = (RR_tire_forces_aligned - RR_tire_forces_projected)[2]

        FL_MR = self.kin_FMU["FL_bump_spring_MR"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        FR_MR = self.kin_FMU["FR_bump_spring_MR"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RL_MR = self.kin_FMU["RL_bump_spring_MR"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]
        RR_MR = self.kin_FMU["RR_bump_spring_MR"](np.array([hwa * 180 / np.pi, heave, pitch, roll]))[0]

        FL_spring_force = FL_MR * FL_Fz_resid
        FR_spring_force = FR_MR * FR_Fz_resid
        RL_spring_force = RL_MR * RL_Fz_resid
        RR_spring_force = RR_MR * RR_Fz_resid

        FL_spring = FL_corner.push_pull_rod.spring
        FR_spring = FR_corner.push_pull_rod.spring
        RL_spring = RL_corner.push_pull_rod.spring
        RR_spring = RR_corner.push_pull_rod.spring

        FL_spring_F_vec = np.array(FL_spring.direction) * FL_spring_force * -1
        FR_spring_F_vec = np.array(FR_spring.direction) * FR_spring_force * -1
        RL_spring_F_vec = np.array(RL_spring.direction) * RL_spring_force * -1
        RR_spring_F_vec = np.array(RR_spring.direction) * RR_spring_force * -1

        FL_spring_seat = np.array(FL_spring.inboard_node.position)
        FR_spring_seat = np.array(FR_spring.inboard_node.position)
        RL_spring_seat = np.array(RL_spring.inboard_node.position)
        RR_spring_seat = np.array(RR_spring.inboard_node.position)

        # === Sprung Mass Forces ===
        sum_F_sprung = FL_spring_F_vec + \
                       FR_spring_F_vec + \
                       RL_spring_F_vec + \
                       RR_spring_F_vec + \
                       FL_tire_forces_projected + \
                       FR_tire_forces_projected + \
                       RL_tire_forces_projected + \
                       RR_tire_forces_projected + \
                       250.56503352247387 * self.sus_data.g
                       # self.sus.sprung_mass * self.sus_data.g  # g = -9.81 m/s^2

        _, _, accZ = sum_F_sprung / self.sus.sprung_mass

        print(accZ)

        # === Sprung Mass Moments ===
        FL_FVIC_wrt_CG = FL_FVIC_pos - vehCG
        FR_FVIC_wrt_CG = FR_FVIC_pos - vehCG
        RL_FVIC_wrt_CG = RL_FVIC_pos - vehCG
        RR_FVIC_wrt_CG = RR_FVIC_pos - vehCG

        FL_spring_wrt_CG = FL_spring_seat - vehCG
        FR_spring_wrt_CG = FR_spring_seat - vehCG
        RL_spring_wrt_CG = RL_spring_seat - vehCG
        RR_spring_wrt_CG = RR_spring_seat - vehCG

        FL_sprung_moment = np.cross(FL_FVIC_wrt_CG, FL_tire_forces_projected) + np.cross(FL_spring_wrt_CG, FL_spring_F_vec)
        FR_sprung_moment = np.cross(FR_FVIC_wrt_CG, FR_tire_forces_projected) + np.cross(FR_spring_wrt_CG, FR_spring_F_vec)
        RL_sprung_moment = np.cross(RL_FVIC_wrt_CG, RL_tire_forces_projected) + np.cross(RL_spring_wrt_CG, RL_spring_F_vec)
        RR_sprung_moment = np.cross(RR_FVIC_wrt_CG, RR_tire_forces_projected) + np.cross(RR_spring_wrt_CG, RR_spring_F_vec)

        sum_M_sprung = FL_sprung_moment + FR_sprung_moment + RL_sprung_moment + RR_sprung_moment
        accRoll, accPitch, _ = np.linalg.solve(self.sus.inertia_tensor, sum_M_sprung)

        # === External Loads ===
        gravity_force = np.array([0, 0, self.sus.total_mass * self.sus_data.g])

        # === Internal Loads ===
        suspension_forces = FL_tire_forces_aligned + \
                            FL_tire_forces_aligned + \
                            FL_tire_forces_aligned + \
                            FL_tire_forces_aligned
        
        suspension_moments = np.cross(FL_cp_wrt_CG, FL_tire_forces_aligned) + \
                             np.cross(FR_cp_wrt_CG, FR_tire_forces_aligned) + \
                             np.cross(RL_cp_wrt_CG, RL_tire_forces_aligned) + \
                             np.cross(RR_cp_wrt_CG, RR_tire_forces_aligned) + \
                             FL_tire_moments_aligned + \
                             FR_tire_moments_aligned + \
                             RL_tire_moments_aligned + \
                             RR_tire_moments_aligned

        # === Sum Forces and Momments ===
        sum_F = suspension_forces + gravity_force
        sum_M = suspension_moments

        accX, accY, _ = sum_F / self.sus.total_mass
        _, _, accYaw = np.linalg.solve(self.sus.inertia_tensor, sum_M)

        return [accX, accY, accZ, accRoll, accPitch, accYaw]

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
                posX     : Longitudinal position (m)
                posY     : Lateral position (m)
                posZ     : Vertical position (m)
                phi      : Roll angle (rad)
                theta    : Pitch angle (rad)
                psi      : Yaw angle (rad)
                velX     : Longitudinal velocity (m/s)
                velY     : Lateral velocity (m/s)
                velZ     : Vertical velocity (m/s)
                velRoll  : Roll velocity (rad/s)
                velPitch : Pitch velocity (rad/s)
                velYaw   : Yaw velocity (rad/s)

            der_x : Tuple[float, float, float]

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
            posX, posY, posZ, phi, theta, psi, velX, velY, velZ, velRoll, velPitch, yawVel = x
            dx_pos, dy_pos, dz_pos, dphi, dtheta, dpsi, accX, accY, accZ, accRoll, accPitch, accYaw = der_x
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
