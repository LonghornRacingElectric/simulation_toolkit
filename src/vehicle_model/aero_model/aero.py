from scipy.interpolate import RBFInterpolator

import pandas as pd
import numpy as np


class Aero:
    def __init__(self, aero_map_path: str) -> None:
        self.map_path = aero_map_path

        # Load CSV
        df = pd.read_csv(self.map_path)

        # Convert radians to degrees
        roll = df["Roll"].to_numpy() * 180 / np.pi
        pitch = df["Pitch"].to_numpy() * 180 / np.pi
        yaw = df["Yaw"].to_numpy() * 180 / np.pi

        # Create independent variable array (N, 3)
        ind_var = np.stack([roll, pitch, yaw], axis=-1)

        # Dependent variables
        CxA_vals = df["CxA"].to_numpy()
        CyA_vals = df["CyA"].to_numpy()
        CzA_vals = df["CzA"].to_numpy()
        MxA_vals = df["MxA"].to_numpy()
        MyA_vals = df["MyA"].to_numpy()
        MzA_vals = df["MzA"].to_numpy()

        # Create interpolators
        self.CxA_interp = RBFInterpolator(ind_var, CxA_vals, kernel='linear', epsilon=1.0)
        self.CyA_interp = RBFInterpolator(ind_var, CyA_vals, kernel='linear', epsilon=1.0)
        self.CzA_interp = RBFInterpolator(ind_var, CzA_vals, kernel='linear', epsilon=1.0)
        self.MxA_interp = RBFInterpolator(ind_var, MxA_vals, kernel='linear', epsilon=1.0)
        self.MyA_interp = RBFInterpolator(ind_var, MyA_vals, kernel='linear', epsilon=1.0)
        self.MzA_interp = RBFInterpolator(ind_var, MzA_vals, kernel='linear', epsilon=1.0)
    
    def eval(self, roll: float, pitch: float, yaw: float, vel: float):
        coeff = 0.5 * 1.225 * vel**2  # dynamic pressure

        # Prepare input as 2D array (shape: [1, 3])
        query_point = np.array([[roll, pitch, yaw]])

        Fx = coeff * self.CxA_interp(query_point)[0]
        Fy = coeff * self.CyA_interp(query_point)[0]
        Fz = coeff * self.CzA_interp(query_point)[0]
        Mx = coeff * self.MxA_interp(query_point)[0]
        My = coeff * self.MyA_interp(query_point)[0]
        Mz = coeff * self.MzA_interp(query_point)[0]

        return Fx, Fy, Fz, Mx, My, Mz


if __name__ == "__main__":
    aer = Aero("./_1_model_inputs/aero_map.csv")
    print(aer.eval(roll=0, pitch=0, yaw=0, vel=15))