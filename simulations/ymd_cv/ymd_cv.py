from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension
from _4_custom_libraries.cache import SISO_cache

from typing import Callable, MutableSequence, Sequence, Set, Tuple
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import numpy as np
import yaml


class YMDConstantVelocity:
    def __init__(self, model_path: str):
        sus_data = SuspensionData(path=model_path)
        self.sus = Suspension(sus_data=sus_data)

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

        self.states = {"x_ddot": [],
                       "y_ddot": [],
                       "yaw_ddot": [],
                       "heave": [],
                       "pitch": [],
                       "roll": []}
        
        for i, beta in enumerate(np.linspace(-self.beta, self.beta, self.refinement)):
            for j, hwa in enumerate(np.linspace(-self.hwa, self.hwa, self.refinement)):
                print(f"Progress | {round(counter / self.refinement**2 * 100, 1)}%", end="\r")
                counter += 1
                # x_ddot, y_ddot, yaw_ddot, heave, pitch, roll = fsolve(self._residual_function, x0=[0, 0, 0, 0, 0, 0], args=[hwa, beta, self.velX])
                
                # x_ddot_lst.append(x_ddot)
                # y_ddot_lst.append(y_ddot)
                # yaw_ddot_lst.append(yaw_ddot)
                # heave_lst.append(heave)
                # pitch_lst.append(pitch)
                # roll_lst.append(roll)

                # delta_lst.append(delta)
                # beta_lst.append(beta)

                # self.steered_angle_iso_lines[j][0] = delta / (3.50 / (2 * np.pi) * 0.0254)
                # self.steered_angle_iso_lines[j][1][i] = y_ddot
                # self.steered_angle_iso_lines[j][2][i] = yaw_ddot
                
                # self.body_slip_iso_lines[i][0] = beta
                # self.body_slip_iso_lines[i][1][j] = y_ddot
                # self.body_slip_iso_lines[i][2][j] = yaw_ddot
    
    def physical_model(self, x: Sequence[float], args: Sequence[float]) -> Sequence[float]:
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

        # Physical system
        turn_radius = velX**2 / accY

        vehVel = velX * np.array([np.cos(beta), np.sin(beta), 0])

        # Adjust acceleration vectors and dependencies
        # vehAccel = np.array([accX, accY, 0])
        # ntbAccel = np.matmul(rotation_matrix(unit_vec=[0, 0, 1], theta=beta), imf_accel)
        # yaw_rate = 0 if ntb_accel[1] == 0 else ntb_accel[1] / np.linalg.norm(imf_velocity)