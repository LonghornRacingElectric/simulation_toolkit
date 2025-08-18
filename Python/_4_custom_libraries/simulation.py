from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension
from vehicle_model.aero_model.aero import Aero

from _4_custom_libraries.misc_math import rotation_matrix

from matplotlib.backends.backend_pdf import PdfPages
from scipy.interpolate import interp1d
from matplotlib.figure import Figure
from scipy.optimize import fsolve
from scipy.integrate import quad
from typing import Sequence
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
import subprocess
import pickle
import os


class Simulation:
    def __init__(self, model_path: str):
        self.sus_data = SuspensionData(path=model_path)
        self.sus = Suspension(sus_data=self.sus_data)
        self.sus_copy = deepcopy(self.sus)

        if not ("kin_FMU.pkl" in os.listdir("./simulations/kin/kin_outputs/")):
            raise Exception("Please run SIM=kin with FMU generation enabled")
        
        with open("./simulations/kin/kin_outputs/kin_FMU.pkl", 'rb') as f:
            self.kin_FMU = pickle.load(f)
        
        self.initialize_funcs()

    def initialize_funcs(self) -> None:
        # Initialize sweeps
        jounce_sweep = np.linspace(-5, 5, 20) * 0.0254
        roll_sweep = np.linspace(-5, 5, 20)
        pitch_sweep = np.linspace(-5, 5, 20)

        # Sweep to create arrays of modal stiffnesses
        Fr_Krs = [self.kin_FMU["Fr_Kr"](np.array([0, 0, 0, roll])) for roll in roll_sweep]
        Rr_Krs = [self.kin_FMU["Rr_Kr"](np.array([0, 0, 0, roll])) for roll in roll_sweep]
        Avg_Kps = [self.kin_FMU["Avg_Kp"](np.array([0, 0, pitch, 0])) for pitch in pitch_sweep]

        FL_wheelrate = [self.kin_FMU["FL_wheelrate"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        FR_wheelrate = [self.kin_FMU["FR_wheelrate"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        RL_wheelrate = [self.kin_FMU["RL_wheelrate"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        RR_wheelrate = [self.kin_FMU["RR_wheelrate"](np.array([0, jounce, 0, 0])) for jounce in jounce_sweep]
        
        Avg_Kh = np.array(FL_wheelrate) + np.array(FR_wheelrate) + np.array(RL_wheelrate) + np.array(RR_wheelrate)

        # Create cubic fits from previous arrays
        Fr_Kr_coeffs = np.polyfit(roll_sweep, Fr_Krs, deg=3).T[0]
        Rr_Kr_coeffs = np.polyfit(roll_sweep, Rr_Krs, deg=3).T[0]
        Avg_Kp_coeffs = np.polyfit(pitch_sweep, Avg_Kps, deg=3).T[0]
        Avg_Kh_coeffs = np.polyfit(jounce_sweep, Avg_Kh, deg=3).T[0]

        # Represent cubic fits as lambda functions
        self.Fr_Kr = lambda x: sum([Fr_Kr_coeffs[i] * x**(3 - i) for i in range(4)])
        self.Rr_Kr = lambda x: sum([Rr_Kr_coeffs[i] * x**(3 - i) for i in range(4)])
        self.Avg_Kp = lambda x: sum([Avg_Kp_coeffs[i] * x**(3 - i) for i in range(4)])
        self.Avg_Kh = lambda x: sum([Avg_Kh_coeffs[i] * x**(3 - i) for i in range(4)])
    
    def get_git_username(self):
        try:
            name = subprocess.check_output(
                ["git", "config", "user.email"], stderr=subprocess.DEVNULL
            ).decode().strip()
            return name if name else "Unknown"
        except Exception:
            return "Unknown"
    
    def get_git_name(self):
        try:
            name = subprocess.check_output(
                ["git", "config", "user.name"], stderr=subprocess.DEVNULL
            ).decode().strip()
            return name if name else "Unknown"
        except Exception:
            return "Unknown"

    def _generate_pdf(self, figs: Sequence[Figure], save_path: str) -> Figure:
        p = PdfPages(save_path)

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()