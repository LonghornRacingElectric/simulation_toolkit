from src.vehicle_model.suspension_model.suspension_data import SuspensionData
from src.vehicle_model.suspension_model.suspension import Suspension
from src._3_custom_libraries.misc_math import rotation_matrix
from src.vehicle_model.aero_model.aero import Aero

from typing import Union, Sequence, Tuple, MutableSequence
from scipy.optimize import fsolve
from scipy.integrate import quad

import numpy as np
import pickle
import os


class YMDConstantVelocity:
    def __init__(self, model_path: str, velX: float, hwa: float, beta: float, refinement: int):
        # Read FMU
        if not ("kin_FMU.pkl" in os.listdir("./src/simulations/kin/kin_outputs/")):
            raise Exception("Please run SIM=kin with FMU generation enabled")
        
        with open("./src/simulations/kin/kin_outputs/kin_FMU.pkl", 'rb') as f:
            self.kin_FMU = pickle.load(f)

        # Simulation parameters
        self.velX = velX
        self.hwa = hwa
        self.beta = beta
        self.refinement = refinement
        
        self.states: dict[str, dict[str, list[Union[float, list[float]]]]] = {"consts": {"velX":        []},
                                                                                 "hwa": {"angle":       [],
                                                                                         "hwa":         [],
                                                                                         "beta":        [],
                                                                                         "accX":        [],
                                                                                         "accY":        [],
                                                                                         "accYaw":      [],
                                                                                         "heave":       [],
                                                                                         "theta":       [],
                                                                                         "phi":         [],
                                                                                         "gamma":       [],
                                                                                         "delta":       [],
                                                                                         "alpha":       [],
                                                                                         "turn_radius": [],
                                                                                         "velX":        []},
                                                                                "beta": {"angle":       [],
                                                                                         "hwa":         [],
                                                                                         "beta":        [],
                                                                                         "accX":        [],
                                                                                         "accY":        [],
                                                                                         "accYaw":      [],
                                                                                         "heave":       [],
                                                                                         "theta":       [],
                                                                                         "phi":         [],
                                                                                         "gamma":       [],
                                                                                         "delta":       [],
                                                                                         "alpha":       [],
                                                                                         "turn_radius": [],
                                                                                         "velX":        []}}

        # Initialize simulation
        self.sus_data: SuspensionData = SuspensionData(path=model_path)
        self.sus: Suspension = Suspension(sus_data=self.sus_data)
        self.initialize_funcs()

        self.turn_radius: float
        self.gamma_vals: MutableSequence
        self.delta_vals: MutableSequence
        self.alpha_vals: MutableSequence

    def run(self) -> dict[str, dict[str, list[Union[float, list[float]]]]]:
        hwa_sweep = np.linspace(-self.hwa, self.hwa, self.refinement) # leave as deg
        beta_sweep = np.linspace(-self.beta, self.beta, self.refinement) * np.pi / 180 # convert to rad
        counter = 0

        # Run simulation        
        for beta in beta_sweep:
            for hwa in hwa_sweep:
                print(f"Progress | {round(counter / self.refinement**2 * 100, 1)}%", end="\r")
                counter += 1

                # Solve system
                accX, accY, accYaw, heave, theta, phi = fsolve(self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX])
                self.physical_model(x=[accX, accY, accYaw, heave, theta, phi], args=[hwa, beta, self.velX])

                # Store results (hwa)
                if hwa in self.states["hwa"]["angle"]:
                    hwa_index = self.states["hwa"]["angle"].index(hwa)

                    self.states["hwa"]["hwa"][hwa_index].append(hwa)
                    self.states["hwa"]["beta"][hwa_index].append(beta * 180 / np.pi)
                    self.states["hwa"]["accX"][hwa_index].append(accX)
                    self.states["hwa"]["accY"][hwa_index].append(accY)
                    self.states["hwa"]["accYaw"][hwa_index].append(accYaw)
                    self.states["hwa"]["heave"][hwa_index].append(heave)
                    self.states["hwa"]["theta"][hwa_index].append(theta)
                    self.states["hwa"]["phi"][hwa_index].append(phi)
                    self.states["hwa"]["gamma"][hwa_index].append([x * 180 / np.pi for x in self.gamma_vals])
                    self.states["hwa"]["delta"][hwa_index].append([x * 180 / np.pi for x in self.delta_vals])
                    self.states["hwa"]["alpha"][hwa_index].append([x * 180 / np.pi for x in self.alpha_vals])
                    self.states["hwa"]["turn_radius"][hwa_index].append(self.turn_radius)
                    self.states["hwa"]["velX"][hwa_index].append(self.velX)
                else:
                    self.states["hwa"]["angle"].append(hwa)
                    
                    self.states["hwa"]["hwa"].append([hwa])
                    self.states["hwa"]["beta"].append([beta * 180 / np.pi])
                    self.states["hwa"]["accX"].append([accX])
                    self.states["hwa"]["accY"].append([accY])
                    self.states["hwa"]["accYaw"].append([accYaw])
                    self.states["hwa"]["heave"].append([heave])
                    self.states["hwa"]["theta"].append([theta])
                    self.states["hwa"]["phi"].append([phi])
                    self.states["hwa"]["gamma"].append([[x * 180 / np.pi for x in self.gamma_vals]])
                    self.states["hwa"]["delta"].append([[x * 180 / np.pi for x in self.delta_vals]])
                    self.states["hwa"]["alpha"].append([[x * 180 / np.pi for x in self.alpha_vals]])
                    self.states["hwa"]["turn_radius"].append([self.turn_radius])
                    self.states["hwa"]["velX"].append([self.velX])
                
                # (beta)
                if beta * 180 / np.pi in self.states["beta"]["angle"]:
                    beta_index = self.states["beta"]["angle"].index(beta * 180 / np.pi)

                    self.states["beta"]["hwa"][beta_index].append(hwa)
                    self.states["beta"]["beta"][beta_index].append(beta * 180 / np.pi)
                    self.states["beta"]["accX"][beta_index].append(accX)
                    self.states["beta"]["accY"][beta_index].append(accY)
                    self.states["beta"]["accYaw"][beta_index].append(accYaw)
                    self.states["beta"]["heave"][beta_index].append(heave)
                    self.states["beta"]["theta"][beta_index].append(theta)
                    self.states["beta"]["phi"][beta_index].append(phi)
                    self.states["beta"]["gamma"][beta_index].append([x * 180 / np.pi for x in self.gamma_vals])
                    self.states["beta"]["delta"][beta_index].append([x * 180 / np.pi for x in self.delta_vals])
                    self.states["beta"]["alpha"][beta_index].append([x * 180 / np.pi for x in self.alpha_vals])
                    self.states["beta"]["turn_radius"][beta_index].append(self.turn_radius)
                    self.states["beta"]["velX"][beta_index].append(self.velX)
                else:
                    self.states["beta"]["angle"].append(beta * 180 / np.pi)

                    self.states["beta"]["hwa"].append([hwa])
                    self.states["beta"]["beta"].append([beta * 180 / np.pi])
                    self.states["beta"]["accX"].append([accX])
                    self.states["beta"]["accY"].append([accY])
                    self.states["beta"]["accYaw"].append([accYaw])
                    self.states["beta"]["heave"].append([heave])
                    self.states["beta"]["theta"].append([theta])
                    self.states["beta"]["phi"].append([phi])
                    self.states["beta"]["gamma"].append([[x * 180 / np.pi for x in self.gamma_vals]])
                    self.states["beta"]["delta"].append([[x * 180 / np.pi for x in self.delta_vals]])
                    self.states["beta"]["alpha"].append([[x * 180 / np.pi for x in self.alpha_vals]])
                    self.states["beta"]["turn_radius"].append([self.turn_radius])
                    self.states["beta"]["velX"].append([self.velX])
        
        self.states["consts"]["velX"].append(self.velX)
        
        return self.states
    
    def physical_model(self, x: Sequence[float], args: Tuple[float, float, float]) -> Sequence[float]:
        # States
        accX = x[0]
        accY = x[1]
        accYaw = x[2]
        heave = x[3]
        theta = x[4]
        phi = x[5]

        # Constants
        hwa = args[0]
        beta = args[1]
        velX = args[2]
        
        self.turn_radius = velX**2 / accY
        vehVel = velX * np.array([np.cos(beta), np.sin(beta), 0])

        # Adjust acceleration vectors and dependencies
        vehAccel = np.array([accX, accY, 0])
        ntbAccel = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=beta), vehAccel)
        velYaw = ntbAccel[1] / np.linalg.norm(vehVel)

        # Get contact patch locations
        FL_cp_x = self.kin_FMU["FL_cp_x"](np.array([hwa, heave, theta, phi]))[0]
        FL_cp_y = self.kin_FMU["FL_cp_y"](np.array([hwa, heave, theta, phi]))[0]
        FL_cp_z = self.kin_FMU["FL_cp_z"](np.array([hwa, heave, theta, phi]))[0]

        FR_cp_x = self.kin_FMU["FR_cp_x"](np.array([hwa, heave, theta, phi]))[0]
        FR_cp_y = self.kin_FMU["FR_cp_y"](np.array([hwa, heave, theta, phi]))[0]
        FR_cp_z = self.kin_FMU["FR_cp_z"](np.array([hwa, heave, theta, phi]))[0]

        RL_cp_x = self.kin_FMU["RL_cp_x"](np.array([hwa, heave, theta, phi]))[0]
        RL_cp_y = self.kin_FMU["RL_cp_y"](np.array([hwa, heave, theta, phi]))[0]
        RL_cp_z = self.kin_FMU["RL_cp_z"](np.array([hwa, heave, theta, phi]))[0]

        RR_cp_x = self.kin_FMU["RR_cp_x"](np.array([hwa, heave, theta, phi]))[0]
        RR_cp_y = self.kin_FMU["RR_cp_y"](np.array([hwa, heave, theta, phi]))[0]
        RR_cp_z = self.kin_FMU["RR_cp_z"](np.array([hwa, heave, theta, phi]))[0]

        self.FL_cp_pos = np.array([FL_cp_x, FL_cp_y, FL_cp_z])
        self.FR_cp_pos = np.array([FR_cp_x, FR_cp_y, FR_cp_z])
        self.RL_cp_pos = np.array([RL_cp_x, RL_cp_y, RL_cp_z])
        self.RR_cp_pos = np.array([RR_cp_x, RR_cp_y, RR_cp_z])

        # Get CG position
        cg_x = self.kin_FMU["veh_CG_x"](np.array([hwa, heave, theta, phi]))[0]
        cg_y = self.kin_FMU["veh_CG_y"](np.array([hwa, heave, theta, phi]))[0]
        cg_z = self.kin_FMU["veh_CG_z"](np.array([hwa, heave, theta, phi]))[0]

        self.cg_pos = np.array([cg_x, cg_y, cg_z])

        # Store contact patch positions
        self.FL_Cp_wrt_cg = self.FL_cp_pos - self.cg_pos
        self.FR_Cp_wrt_cg = self.FR_cp_pos - self.cg_pos
        self.RL_Cp_wrt_cg = self.RL_cp_pos - self.cg_pos
        self.RR_Cp_wrt_cg = self.RR_cp_pos - self.cg_pos

        # Inclination angles
        FL_gamma = self.kin_FMU["FL_gamma"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180
        FR_gamma = self.kin_FMU["FR_gamma"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180
        RL_gamma = self.kin_FMU["RL_gamma"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180
        RR_gamma = self.kin_FMU["RR_gamma"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180

        self.gamma_vals = [FL_gamma, FR_gamma, RL_gamma, RR_gamma]

        # Delta angles
        FL_delta = self.kin_FMU["FL_delta"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180
        FR_delta = self.kin_FMU["FR_delta"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180
        RL_delta = self.kin_FMU["RL_delta"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180
        RR_delta = self.kin_FMU["RR_delta"](np.array([hwa, heave, theta, phi]))[0] * np.pi / 180

        self.delta_vals = [FL_delta, FR_delta, RL_delta, RR_delta]

        # Wheel velocities
        FL_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.FL_Cp_wrt_cg)
        FR_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.FR_Cp_wrt_cg)
        RL_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.RL_Cp_wrt_cg)
        RR_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.RR_Cp_wrt_cg)

        # Slip angles
        FL_alpha = FL_delta - np.arctan2(FL_velocity[1], FL_velocity[0])
        FR_alpha = FR_delta - np.arctan2(FR_velocity[1], FR_velocity[0])
        RL_alpha = RL_delta - np.arctan2(RL_velocity[1], RL_velocity[0])
        RR_alpha = RR_delta - np.arctan2(RR_velocity[1], RR_velocity[0])

        self.alpha_vals = [FL_alpha, FR_alpha, RL_alpha, RR_alpha]

        # Corner jounces
        FL_jounce = self.kin_FMU["FL_wheel_jounce"](np.array([hwa, heave, theta, phi]))[0]
        FR_jounce = self.kin_FMU["FR_wheel_jounce"](np.array([hwa, heave, theta, phi]))[0]
        RL_jounce = self.kin_FMU["RL_wheel_jounce"](np.array([hwa, heave, theta, phi]))[0]
        RR_jounce = self.kin_FMU["RR_wheel_jounce"](np.array([hwa, heave, theta, phi]))[0]

        FL_wheelrate = lambda x: self.FL_quarter_car.push_pull_rod.spring.compliance / self.FL_spring_MR_eqn(x)**2
        FR_wheelrate = lambda x: self.FR_quarter_car.push_pull_rod.spring.compliance / self.FR_spring_MR_eqn(x)**2
        RL_wheelrate = lambda x: self.RL_quarter_car.push_pull_rod.spring.compliance / self.RL_spring_MR_eqn(x)**2
        RR_wheelrate = lambda x: self.RR_quarter_car.push_pull_rod.spring.compliance / self.RR_spring_MR_eqn(x)**2

        # Static weights
        FL_static_weight = self.FL_quarter_car.static_weight
        FR_static_weight = self.FR_quarter_car.static_weight
        RL_static_weight = self.RL_quarter_car.static_weight
        RR_static_weight = self.RR_quarter_car.static_weight

        # Inelastic load transfer
        wheelbase = abs((self.FL_cp_pos - self.RL_cp_pos)[0]) # Assume the same left to right. TODO: add left/right independently
        front_track = abs((self.FL_cp_pos - self.FR_cp_pos)[1])
        rear_track = abs((self.RL_cp_pos - self.RR_cp_pos)[1])
        cg_height = self.cg_pos[2]

        ### lateral ###
        Fr_roll_stiffness = 1 / 4 * front_track**2 * (FL_wheelrate(FL_jounce) + FR_wheelrate(FR_jounce))
        Rr_roll_stiffness = 1 / 4 * rear_track**2 * (RL_wheelrate(RL_jounce) + RR_wheelrate(RR_jounce))
        total_roll_stiffness = Fr_roll_stiffness + Rr_roll_stiffness

        FL_delta_lat = -self.sus.total_mass * accY * cg_height / front_track * (Fr_roll_stiffness / total_roll_stiffness)
        FR_delta_lat = self.sus.total_mass * accY * cg_height / front_track * (Fr_roll_stiffness / total_roll_stiffness)
        RL_delta_lat = -self.sus.total_mass * accY * cg_height / rear_track * (Rr_roll_stiffness / total_roll_stiffness)
        RR_delta_lat = self.sus.total_mass * accY * cg_height / rear_track * (Rr_roll_stiffness / total_roll_stiffness)

        ### longitudinal ###
        FL_delta_long = -(self.sus.total_mass * accX * cg_height / wheelbase) / 2
        FR_delta_long = -(self.sus.total_mass * accX * cg_height / wheelbase) / 2
        RL_delta_long = (self.sus.total_mass * accX * cg_height / wheelbase) / 2
        RR_delta_long = (self.sus.total_mass * accX * cg_height / wheelbase) / 2

        FL_delta_inelastic = FL_delta_lat + FL_delta_long
        FR_delta_inelastic = FR_delta_lat + FR_delta_long
        RL_delta_inelastic = RL_delta_lat + RL_delta_long
        RR_delta_inelastic = RR_delta_lat + RR_delta_long

        # Elastic load transfer
        FL_delta_elastic = quad(func=FL_wheelrate, a=min([0, FL_jounce]), b=max([0, FL_jounce]))[0] * np.sign(FL_jounce)
        FR_delta_elastic = quad(func=FR_wheelrate, a=min([0, FR_jounce]), b=max([0, FR_jounce]))[0] * np.sign(FR_jounce)
        RL_delta_elastic = quad(func=RL_wheelrate, a=min([0, RL_jounce]), b=max([0, RL_jounce]))[0] * np.sign(RL_jounce)
        RR_delta_elastic = quad(func=RR_wheelrate, a=min([0, RR_jounce]), b=max([0, RR_jounce]))[0] * np.sign(RR_jounce)

        FL_Fz = FL_static_weight + FL_delta_inelastic + FL_delta_elastic
        FR_Fz = FR_static_weight + FR_delta_inelastic + FR_delta_elastic
        RL_Fz = RL_static_weight + RL_delta_inelastic + RL_delta_elastic
        RR_Fz = RR_static_weight + RR_delta_inelastic + RR_delta_elastic

        self.FL_tire_output = self.FL_quarter_car.tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)
        self.FR_tire_output = self.FR_quarter_car.tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)
        self.RL_tire_output = self.RL_quarter_car.tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)
        self.RR_tire_output = self.RR_quarter_car.tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)

        self.FL_tire_forces = self.FL_tire_output[0:3]
        self.FR_tire_forces = self.FR_tire_output[0:3]
        self.RL_tire_forces = self.RL_tire_output[0:3]
        self.RR_tire_forces = self.RR_tire_output[0:3]

        self.FL_tire_moments = self.FL_tire_output[3:]
        self.FR_tire_moments = self.FR_tire_output[3:]
        self.RL_tire_moments = self.RL_tire_output[3:]
        self.RR_tire_moments = self.RR_tire_output[3:]

        FL_vel_ang = np.arctan2(FL_velocity[1], FL_velocity[0])
        FR_vel_ang = np.arctan2(FR_velocity[1], FR_velocity[0])
        RL_vel_ang = np.arctan2(RL_velocity[1], RL_velocity[0])
        RR_vel_ang = np.arctan2(RR_velocity[1], RR_velocity[0])

        self.FL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_vel_ang), self.FL_tire_forces)
        self.FR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_vel_ang), self.FR_tire_forces)
        self.RL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_vel_ang), self.RL_tire_forces)
        self.RR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_vel_ang), self.RR_tire_forces)

        self.FL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_vel_ang), self.FL_tire_moments)
        self.FR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_vel_ang), self.FR_tire_moments)
        self.RL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_vel_ang), self.RL_tire_moments)
        self.RR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_vel_ang), self.RR_tire_moments)

        ###############################################
        ########### Aero Forces and Moments ###########
        ###############################################

        aero = Aero("./src/_1_model_inputs/aero_map.csv")

        aero_loads = aero.eval(roll=phi, pitch=theta, yaw=beta * 180 / np.pi, vel=velX)

        ###############################################
        ######## Calculate Forces and Moments #########
        ###############################################

        gravity_force = np.array([0, 0, self.sus.total_mass * 9.81])

        suspension_forces = self.FL_tire_forces_aligned + self.FR_tire_forces_aligned + self.RL_tire_forces_aligned + self.RR_tire_forces_aligned
        suspension_moments = np.cross(self.FL_Cp_wrt_cg, self.FL_tire_forces_aligned) + np.cross(self.FR_Cp_wrt_cg, self.FR_tire_forces_aligned) + \
                             np.cross(self.RL_Cp_wrt_cg, self.RL_tire_forces_aligned) + np.cross(self.RR_Cp_wrt_cg, self.RR_tire_forces_aligned) # + \
                            #  self.FL_tire_moments_aligned + self.FR_tire_moments_aligned + self.RL_tire_moments_aligned + self.RR_tire_moments_aligned

        aero_forces = aero_loads[:3]
        aero_moments = aero_loads[3:]

        vehicle_centric_forces = -1 * gravity_force + np.array(suspension_forces) + np.array(aero_forces)
        vehicle_centric_moments = suspension_moments + aero_moments

        ###################################################
        ### Calculate Forces and Moments from Iteration ###
        ###################################################

        # Initialize translational acceleration
        vehTransAccel = np.array([accX, accY, 0])

        # Sum of forces and moments
        sum_force = self.sus.total_mass * vehTransAccel
        sum_moment = (np.array([[119.8, 0, 0], [0, 33.4, 0], [0, 0, 108.2]]) @ np.array([[0], [0], [accYaw]])).T[0]

        force_residuals = vehicle_centric_forces - sum_force
        moment_residuals = vehicle_centric_moments - sum_moment

        return np.array([*force_residuals, *moment_residuals])

    def initialize_funcs(self) -> None:
        # Initialize sweeps
        jounce_sweep = np.linspace(-5, 5, 20) * 0.0254
        roll_sweep = np.linspace(-5, 5, 20)

        # Sweep to create arrays of spring and stabar MRs
        FL_spring_MRs = [self.kin_FMU["FL_bump_spring_MR"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        FR_spring_MRs = [self.kin_FMU["FR_bump_spring_MR"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        RL_spring_MRs = [self.kin_FMU["RL_bump_spring_MR"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        RR_spring_MRs = [self.kin_FMU["RR_bump_spring_MR"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]

        Fr_stabar_MRs = [self.kin_FMU["Fr_roll_stabar_MR"](np.array([0, 0, 0, roll])) for roll in roll_sweep]
        Rr_stabar_MRs = [self.kin_FMU["Rr_roll_stabar_MR"](np.array([0, 0, 0, roll])) for roll in roll_sweep]

        # Create cubic fits from previous arrays
        FL_spring_MR_coeffs = np.polyfit(jounce_sweep, FL_spring_MRs, deg=3).T[0]
        FR_spring_MR_coeffs = np.polyfit(jounce_sweep, FR_spring_MRs, deg=3).T[0]
        RL_spring_MR_coeffs = np.polyfit(jounce_sweep, RL_spring_MRs, deg=3).T[0]
        RR_spring_MR_coeffs = np.polyfit(jounce_sweep, RR_spring_MRs, deg=3).T[0]
        Fr_stabar_MR_coeffs = np.polyfit(roll_sweep, Fr_stabar_MRs, deg=3).T[0]
        Rr_stabar_MR_coeffs = np.polyfit(roll_sweep, Rr_stabar_MRs, deg=3).T[0]

        # Represent cubic fits as lambda functions
        self.FL_spring_MR_eqn = lambda x: sum([FL_spring_MR_coeffs[i] * x**(3 - i) for i in range(4)])
        self.FR_spring_MR_eqn = lambda x: sum([FR_spring_MR_coeffs[i] * x**(3 - i) for i in range(4)])
        self.RL_spring_MR_eqn = lambda x: sum([RL_spring_MR_coeffs[i] * x**(3 - i) for i in range(4)])
        self.RR_spring_MR_eqn = lambda x: sum([RR_spring_MR_coeffs[i] * x**(3 - i) for i in range(4)])

        self.Fr_stabar_MR_eqn = lambda x: sum([Fr_stabar_MR_coeffs[i] * x**(3 - i) for i in range(4)])
        self.Rr_stabar_MR_eqn = lambda x: sum([Rr_stabar_MR_coeffs[i] * x**(3 - i) for i in range(4)])

        # Store quarter car models
        self.FL_quarter_car = self.sus.FL_quarter_car
        self.FR_quarter_car = self.sus.FR_quarter_car
        self.RL_quarter_car = self.sus.RL_quarter_car
        self.RR_quarter_car = self.sus.RR_quarter_car