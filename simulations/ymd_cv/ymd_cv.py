from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension
from _4_custom_libraries.misc_math import rotation_matrix
from _4_custom_libraries.cache import SISO_cache

from typing import Callable, MutableSequence, Sequence, Set, Tuple
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from scipy.optimize import fsolve, minimize, root
from scipy.integrate import quad
import matplotlib.pyplot as plt
from copy import deepcopy
import numpy as np
import yaml


class YMDConstantVelocity:
    def __init__(self, model_path: str):
        sus_data = SuspensionData(path=model_path)
        self.sus = Suspension(sus_data=sus_data)
        self.sus_copy = deepcopy(self.sus)

        self.FL_spring_MR, self.FR_spring_MR, self.RL_spring_MR, self.RR_spring_MR = self.spring_MRs
        self.Fr_stabar_MR_rot, self.Rr_stabar_MR_rot = self.stabar_MRs

        roll_n_steps = 5
        counter = 0
        
        with open("./simulations/ymd_cv/ymd_cv_inputs/ymd_cv.yml") as f:
            try:
                self.config : dict[str, dict[str, dict]] = yaml.safe_load(f)
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)
        
        self.outputs: Set[str] = set([key[key.index("_") + 1:] for key in self.sus.state.keys()])

        # Physical parameters
        self.velX = self.config["Physical Parameters"]["Velocity"]
        self.hwa = self.config["Physical Parameters"]["HandwheelAngle Sweep"]
        self.beta = self.config["Physical Parameters"]["SideslipAngle Sweep"]

        # Simulation parameters
        self.refinement = self.config["Simulation Parameters"]["Refinement"]

        self.states = {"accX": [],
                       "accY": [],
                       "accYaw": [],
                       "heave": [],
                       "theta": [],
                       "phi": []}
        
        self.sys_input = {"hwa": [],
                          "beta": []}
        
        self.body_slip_iso_lines = [[0, [0] * self.refinement, [0] * self.refinement] for _ in range(self.refinement)]
        self.steered_angle_iso_lines = [[0, [0] * self.refinement, [0] * self.refinement] for _ in range(self.refinement)]
        
        self.base_state = []
        self.min_state = [0, 0, 0, 0, 0, 0]
        for i, beta in enumerate(np.linspace(-self.beta, self.beta, self.refinement) * np.pi / 180):
            for j, hwa in enumerate(np.linspace(-self.hwa, self.hwa, self.refinement)):
                print(f"Progress | {round(counter / self.refinement**2 * 100, 1)}%", end="\n")
                counter += 1
                self.min_resid = 1e9
                if j == 0 and self.base_state:
                    # accX, accY, accYaw, heave, theta, phi = fsolve(self.physical_model, x0=self.base_state, args=[hwa, beta, self.velX])
                    accX, accY, accYaw, heave, theta, phi = minimize(fun=self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX],
                                                                 bounds=[[-1, 1], [-3, 3], [None, None], [-2 * 0.0254, 2 * 0.0254], [-3, 3], [-3, 3]], options={"maxiterint": 100},
                                                                 method="Nelder-Mead").x
                elif j == 0:
                    # accX, accY, accYaw, heave, theta, phi = fsolve(self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX])
                    accX, accY, accYaw, heave, theta, phi = minimize(fun=self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX],
                                                                 bounds=[[-1, 1], [-3, 3], [None, None], [-2 * 0.0254, 2 * 0.0254], [-3, 3], [-3, 3]], options={"maxiterint": 100},
                                                                 method="Nelder-Mead").x
                    self.base_state = deepcopy(self.min_state)
                else:
                    # accX, accY, accYaw, heave, theta, phi = fsolve(self.physical_model, x0=self.min_state, args=[hwa, beta, self.velX])
                    accX, accY, accYaw, heave, theta, phi = minimize(fun=self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX],
                                                                 bounds=[[-1, 1], [-3, 3], [None, None], [-2 * 0.0254, 2 * 0.0254], [-3, 3], [-3, 3]], options={"maxiterint": 100},
                                                                 method="Nelder-Mead").x
                # accX, accY, accYaw, heave, theta, phi = minimize(fun=self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX],
                #                                                  bounds=[[-1, 1], [-3, 3], [None, None], [-2 * 0.0254, 2 * 0.0254], [-3, 3], [-3, 3]], options={"maxiterint": 100},
                #                                                  method="Nelder-Mead").x
                # accX, accY, accYaw, heave, theta, phi = root(lambda x: self.physical_model(x, hwa, beta, self.velX), x0=[0, 0, 0, 0, 0, 0], options={"maxiter": 500}).x
                
                # self.min_resid = 1e9
                # self.min_state = []
                # fsolve(func=self.physical_model, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX])

                if not self.min_state:
                    continue

                accX, accY, accYaw, heave, theta, phi = self.min_state

                # resid = np.linalg.norm(self.physical_model(x=[accX, accY, accYaw, heave, theta, phi], args=[hwa, beta, self.velX]))

                if self.min_resid < 1000:
                    print("\n")
                    print(f"accX: {accX / 9.81}")
                    print(f"accY: {accY / 9.81}")
                    print(f"accYaw: {accYaw}")
                    print(f"heave: {heave / 0.0254}")
                    print(f"pitch: {theta}")
                    print(f"roll: {phi}")
                    print()

                    self.states["accX"].append(accX)
                    self.states["accY"].append(accY)
                    self.states["accYaw"].append(accYaw)
                    self.states["heave"].append(heave)
                    self.states["theta"].append(theta)
                    self.states["phi"].append(phi)

                    self.sys_input["hwa"].append(hwa)
                    self.sys_input["beta"].append(beta)

                    self.steered_angle_iso_lines[j][0] = hwa / 360 * 0.0889
                    self.steered_angle_iso_lines[j][1][i] = accY
                    self.steered_angle_iso_lines[j][2][i] = accYaw
                    
                    self.body_slip_iso_lines[i][0] = beta
                    self.body_slip_iso_lines[i][1][j] = accY
                    self.body_slip_iso_lines[i][2][j] = accYaw
                
        self.plot()
    
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

        # a_c = v^2 / r
        if abs(accY) < 1e-4:
            turn_radius = np.inf
        else:
            turn_radius = velX**2 / accY

        vehVel = velX * np.array([np.cos(beta), np.sin(beta), 0])

        # Adjust acceleration vectors and dependencies
        vehAccel = np.array([accX, accY, 0])
        ntbAccel = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=beta), vehAccel)
        velYaw = ntbAccel[1] / np.linalg.norm(vehVel + 1e-6)

        # If you don't trust this math ^^^ (velYaw), check out the derivation below!
        # w = v / r
        # a = v^2 / r
        # a = v^2 / (v / w)
        # a = wv -> w = a / v

        # Store tire objects
        FL_tire = self.sus.FL_quarter_car.tire
        FR_tire = self.sus.FR_quarter_car.tire
        RL_tire = self.sus.RL_quarter_car.tire
        RR_tire = self.sus.RR_quarter_car.tire

        # Store suspension corner objects
        FL_quarter_car = self.sus.FL_quarter_car
        FR_quarter_car = self.sus.FR_quarter_car
        RL_quarter_car = self.sus.RL_quarter_car
        RR_quarter_car = self.sus.RR_quarter_car

        # Apply modal displacements
        self.sus.steer(hwa=hwa)
        self.sus.heave(heave=heave)
        self.sus.pitch(pitch=theta)
        self.sus.roll(roll=phi)

        # Get CG position
        self.cg_pos = np.array(self.sus.veh_CG)

        # Store contact patch positions
        self.FL_Cp_wrt_cg = np.array(FL_quarter_car.tire.contact_patch.position) - self.cg_pos
        self.FR_Cp_wrt_cg = np.array(FR_quarter_car.tire.contact_patch.position) - self.cg_pos
        self.RL_Cp_wrt_cg = np.array(RL_quarter_car.tire.contact_patch.position) - self.cg_pos
        self.RR_Cp_wrt_cg = np.array(RR_quarter_car.tire.contact_patch.position) - self.cg_pos
        
        # Inclination anglesheave
        FL_gamma = self.sus.FL_gamma
        FR_gamma = self.sus.FR_gamma
        RL_gamma = self.sus.RL_gamma
        RR_gamma = self.sus.RR_gamma

        # Delta angles
        FL_delta = self.sus.FL_delta
        FR_delta = self.sus.FR_delta
        RL_delta = self.sus.RL_delta
        RR_delta = self.sus.RR_delta

        # Wheel velocities
        FL_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.FL_Cp_wrt_cg)
        FR_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.FR_Cp_wrt_cg)
        RL_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.RL_Cp_wrt_cg)
        RR_velocity = vehVel + np.cross(np.array([0, 0, velYaw]), self.RR_Cp_wrt_cg)

        print("deltas")
        print(FL_delta * 180 / np.pi)
        print(FR_delta * 180 / np.pi)
        print(RL_delta * 180 / np.pi)
        print(RR_delta * 180 / np.pi)

        # Slip angles
        FL_alpha = np.clip(FL_delta - np.arctan2(FL_velocity[1], FL_velocity[0]), -np.pi/6, np.pi/6)
        FR_alpha = np.clip(FR_delta - np.arctan2(FR_velocity[1], FR_velocity[0]), -np.pi/6, np.pi/6)
        RL_alpha = np.clip(RL_delta - np.arctan2(RL_velocity[1], RL_velocity[0]), -np.pi/6, np.pi/6)
        RR_alpha = np.clip(RR_delta - np.arctan2(RR_velocity[1], RR_velocity[0]), -np.pi/6, np.pi/6)

        print("alphas")
        print(FL_alpha * 180 / np.pi)
        print(FR_alpha * 180 / np.pi)
        print(RL_alpha * 180 / np.pi)
        print(RR_alpha * 180 / np.pi)

        # Corner jounces
        FL_jounce = FL_quarter_car.wheel_jounce
        FR_jounce = FR_quarter_car.wheel_jounce
        RL_jounce = RL_quarter_car.wheel_jounce
        RR_jounce = RR_quarter_car.wheel_jounce

        FL_wheelrate = lambda x: FL_quarter_car.push_pull_rod.spring.compliance / self.FL_spring_MR(x)**2
        FR_wheelrate = lambda x: FR_quarter_car.push_pull_rod.spring.compliance / self.FR_spring_MR(x)**2
        RL_wheelrate = lambda x: RL_quarter_car.push_pull_rod.spring.compliance / self.RL_spring_MR(x)**2
        RR_wheelrate = lambda x: RR_quarter_car.push_pull_rod.spring.compliance / self.RR_spring_MR(x)**2

        # Static weights
        FL_static_weight = FL_quarter_car.static_weight
        FR_static_weight = FR_quarter_car.static_weight
        RL_static_weight = RL_quarter_car.static_weight
        RR_static_weight = RR_quarter_car.static_weight

        # Inelastic load transfer
        wheelbase = (FL_quarter_car.tire.contact_patch - RL_quarter_car.tire.contact_patch)[0] # Assume the same left to right. TODO: add left/right independently
        front_track = (FL_quarter_car.tire.contact_patch - FR_quarter_car.tire.contact_patch)[1]
        rear_track = (RL_quarter_car.tire.contact_patch - RR_quarter_car.tire.contact_patch)[1]
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

        # if FL_Fz < 0:
        #     FL_Fz = 0
        # if FR_Fz < 0:
        #     FR_Fz = 0
        # if RL_Fz < 0:
        #     RL_Fz = 0
        # if RR_Fz < 0:
        #     RR_Fz = 0

        # FL_Fz = FL_static_weight + FL_delta_elastic
        # FR_Fz = FR_static_weight + FR_delta_elastic
        # RL_Fz = RL_static_weight + RL_delta_elastic
        # RR_Fz = RR_static_weight + RR_delta_elastic

        # print(FL_Fz, FR_Fz, RL_Fz, RR_Fz)

        # Tire loads
        if FL_Fz > 0:
            self.FL_tire_output = FL_tire.tire_eval(FZ=FL_Fz, alpha=FL_alpha, kappa=0, gamma=FL_gamma)
        else:
            self.FL_tire_output = np.array([0, 0, 0, 0, 0, 0])
        if FR_Fz > 0:
            self.FR_tire_output = FR_tire.tire_eval(FZ=FR_Fz, alpha=FR_alpha, kappa=0, gamma=FR_gamma)
        else:
            self.FR_tire_output = np.array([0, 0, 0, 0, 0, 0])
        if RL_Fz > 0:
            self.RL_tire_output = RL_tire.tire_eval(FZ=RL_Fz, alpha=RL_alpha, kappa=0, gamma=RL_gamma)
        else:
            self.RL_tire_output = np.array([0, 0, 0, 0, 0, 0])
        if RR_Fz > 0:
            self.RR_tire_output = RR_tire.tire_eval(FZ=RR_Fz, alpha=RR_alpha, kappa=0, gamma=RR_gamma)
        else:
            self.RR_tire_output = np.array([0, 0, 0, 0, 0, 0])

        self.FL_tire_forces = self.FL_tire_output[0:3]
        self.FR_tire_forces = self.FR_tire_output[0:3]
        self.RL_tire_forces = self.RL_tire_output[0:3]
        self.RR_tire_forces = self.RR_tire_output[0:3]

        self.FL_tire_moments = self.FL_tire_output[3:]
        self.FR_tire_moments = self.FR_tire_output[3:]
        self.RL_tire_moments = self.RL_tire_output[3:]
        self.RR_tire_moments = self.RR_tire_output[3:]

        self.FL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_alpha), self.FL_tire_forces)
        self.FR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_alpha), self.FR_tire_forces)
        self.RL_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_alpha), self.RL_tire_forces)
        self.RR_tire_forces_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_alpha), self.RR_tire_forces)

        self.FL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FL_alpha), self.FL_tire_moments)
        self.FR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=FR_alpha), self.FR_tire_moments)
        self.RL_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RL_alpha), self.RL_tire_moments)
        self.RR_tire_moments_aligned = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=RR_alpha), self.RR_tire_moments)

        ###############################################
        ########### Aero Forces and Moments ###########
        ###############################################

        # aero_loads = self.vehicle.aero_model.force_props(roll=roll, pitch=pitch, body_slip=beta, heave=heave, velocity=velocity)

        # aero_loads = self.vehicle.aero_model.force_props(roll=roll, pitch=pitch, body_slip=beta, heave=heave, velocity=velocity)

        # aero_forces = aero_loads[:3]
        # aero_FAP = aero_loads[3:]

        # CG_wrt_CoP = aero_FAP - self.cg_pos

        ###############################################
        ######## Calculate Forces and Moments #########
        ###############################################

        gravity_force = np.array([0, 0, self.sus.total_mass * 9.81])

        suspension_forces = self.FL_tire_forces_aligned + self.FR_tire_forces_aligned + self.RL_tire_forces_aligned + self.RR_tire_forces_aligned
        suspension_moments = np.cross(self.FL_Cp_wrt_cg, self.FL_tire_forces_aligned) + np.cross(self.FR_Cp_wrt_cg, self.FR_tire_forces_aligned) + \
                             np.cross(self.RL_Cp_wrt_cg, self.RL_tire_forces_aligned) + np.cross(self.RR_Cp_wrt_cg, self.RR_tire_forces_aligned) + \
                             self.FL_tire_moments + self.FR_tire_moments + self.RL_tire_moments + self.RR_tire_moments

        # aero_forces = aero_forces
        # aero_moments = np.cross(CG_wrt_CoP, aero_forces)

        # aero_forces = 0
        # aero_moments = 0

        vehicle_centric_forces = -1 * gravity_force + np.array(suspension_forces) # + aero_forces
        vehicle_centric_moments = suspension_moments # + aero_moments

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

        residuals = np.array([*force_residuals, *moment_residuals])

        resid_norm = np.linalg.norm(residuals)
        print(resid_norm)
        if resid_norm < self.min_resid:
            self.min_resid = resid_norm
            self.min_state = [accX, accY, accYaw, heave, theta, phi]

        return resid_norm

    @property
    def spring_MRs(self) -> Tuple[Callable, Callable, Callable, Callable]:
        FL_spring_MRs = []
        FR_spring_MRs = []
        RL_spring_MRs = []
        RR_spring_MRs = []

        jounce_sweep = np.linspace(-5 * 0.0254, 5 * 0.0254, 10)
        for jounce in jounce_sweep:
            self.sus_copy.heave(heave=None)
            self.sus_copy.heave(heave=jounce)

            FL_spring_MRs.append(self.sus_copy.state["FL_spring_MR"])
            FR_spring_MRs.append(self.sus_copy.state["FR_spring_MR"])
            RL_spring_MRs.append(self.sus_copy.state["RL_spring_MR"])
            RR_spring_MRs.append(self.sus_copy.state["RR_spring_MR"])

        FL_coeffs = np.polyfit(jounce_sweep, FL_spring_MRs, deg=3)
        FR_coeffs = np.polyfit(jounce_sweep, FR_spring_MRs, deg=3)
        RL_coeffs = np.polyfit(jounce_sweep, RL_spring_MRs, deg=3)
        RR_coeffs = np.polyfit(jounce_sweep, RR_spring_MRs, deg=3)

        FL_eqn = lambda x: FL_coeffs[0] * x**3 + FL_coeffs[1] * x**2 + FL_coeffs[2] * x + FL_coeffs[3]
        FR_eqn = lambda x: FR_coeffs[0] * x**3 + FR_coeffs[1] * x**2 + FR_coeffs[2] * x + FR_coeffs[3]
        RL_eqn = lambda x: RL_coeffs[0] * x**3 + RL_coeffs[1] * x**2 + RL_coeffs[2] * x + RL_coeffs[3]
        RR_eqn = lambda x: RR_coeffs[0] * x**3 + RR_coeffs[1] * x**2 + RR_coeffs[2] * x + RR_coeffs[3]

        return (FL_eqn, FR_eqn, RL_eqn, RR_eqn)
    
    @property
    def stabar_MRs(self) -> Tuple[Callable, Callable]:
        Fr_stabar_MRs_rot = []
        Rr_stabar_MRs_rot = []

        roll_sweep = np.linspace(-5, 5, 10)
        for roll in roll_sweep:
            self.sus_copy.heave(heave=None)
            self.sus_copy.roll(roll=roll)

            Fr_stabar_MRs_rot.append(self.sus_copy.state["Fr_stabar_MR_rot"])
            Rr_stabar_MRs_rot.append(self.sus_copy.state["Rr_stabar_MR_rot"])
        
        Fr_coeffs = np.polyfit(roll_sweep, Fr_stabar_MRs_rot, deg=3)
        Rr_coeffs = np.polyfit(roll_sweep, Rr_stabar_MRs_rot, deg=3)

        Fr_eqn = lambda x: Fr_coeffs[0] * x**3 + Fr_coeffs[1] * x**2 + Fr_coeffs[2] * x + Fr_coeffs[3]
        Rr_eqn = lambda x: Rr_coeffs[0] * x**3 + Rr_coeffs[1] * x**2 + Rr_coeffs[2] * x + Rr_coeffs[3]

        return (Fr_eqn, Rr_eqn)
    
    def plot(self):
        fig = plt.figure()
        fig.set_size_inches(w=11, h=8.5)
        ax = fig.gca()

        ax.set_title(f"Constant Velocity: {self.velX} m/s | Yaw Acceleration vs Lateral Acceleration")

        ax.set_xlabel("Lateral Acceleration (m/s^2)")
        ax.set_ylabel("Yaw Acceleration (rad/s^2)")
        ax.axhline(c="gray", linewidth=0.5)
        ax.axvline(c="gray", linewidth=0.5)

        mp = self.refinement // 2

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
        
        custom_leg = [Line2D([0], [0], color='blue', lw=2),
                Line2D([0], [0], color='red', lw=2)]

        ax.legend(custom_leg, ["Constant δ (HWA)", "Constant β"])
        ax.grid()
        
        fig.savefig(f"./simulations/ymd_cv/ymd_cv_outputs/ymd_{self.velX}_mps.png")
        
        plt.show()