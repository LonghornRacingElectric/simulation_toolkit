from vehicle_model._assets.interp import interp3d
import pandas as pd
import numpy as np


class AeroModel:
    """
    ## Aero Model

    Designed to model aerodynamic forces based on sprung-mass state and environment

    ###### Note, all conventions comply with SAE-J670 Z-up

    Parameters
    ----------
    csv_path : str
        File path to aero map .csv
    air_density : float
        Density of air in kg/m^3
    """
    def __init__(self, csv_path: str, air_density: float) -> None:
            
        self.parameters = locals().copy()
        del self.parameters['self']

        self.aero_map = pd.read_csv(csv_path)
        self.air_density = air_density

        # Inputs
        self.roll_vals = np.array(self.aero_map['Roll'])
        self.pitch_vals = np.array(self.aero_map['Pitch'])
        self.yaw_vals = np.array(self.aero_map['Yaw'])
        
        # Force outputs
        self.CxA_vals = np.array(self.aero_map['CxA'])
        self.CyA_vals = np.array(self.aero_map['CyA'])
        self.CzA_vals = np.array(self.aero_map['CzA'])
        
        # Moment outputs
        self.MxA_vals = np.array(self.aero_map['MxA'])
        self.MyA_vals = np.array(self.aero_map['MyA'])
        self.MzA_vals = np.array(self.aero_map['MzA'])

        # Output interpolation objects

        # Forces
        self.CxA = interp3d(x=self.roll_vals, y=self.pitch_vals, z=self.yaw_vals, v=self.CxA_vals)
        self.CyA = interp3d(x=self.roll_vals, y=self.pitch_vals, z=self.yaw_vals, v=self.CyA_vals)
        self.CzA = interp3d(x=self.roll_vals, y=self.pitch_vals, z=self.yaw_vals, v=self.CzA_vals)

        # Moments
        self.MxA = interp3d(x=self.roll_vals, y=self.pitch_vals, z=self.yaw_vals, v=self.MxA_vals)
        self.MyA = interp3d(x=self.roll_vals, y=self.pitch_vals, z=self.yaw_vals, v=self.MyA_vals)
        self.MzA = interp3d(x=self.roll_vals, y=self.pitch_vals, z=self.yaw_vals, v=self.MzA_vals)

    def force_props(self, roll: float, pitch: float, body_slip: float, heave: float, velocity: float):
        """
        ## Force Properties

        Calculate the force properties (forces and force application point) from aerodynamic effects

        Parameters
        ----------
        roll : float
            vehicle roll in radians
        pitch : float
            vehicle pitch in radians
        body_slip : float
            vehicle body slip in radians
        heave : float
            vehicle heave in meters
        velocity : float
            vehicle velocity in m/s

        Returns
        -------
        np.ndarray
            Numpy array of the form [Fx, Fy, Fz, CoPx, CoPy, CoPz] (N and m, respectively)
        """

        p_dyn = 1/2 * self.air_density * velocity**2

        Fx = self.CxA(x=roll, y=pitch, z=body_slip) * p_dyn
        Fy = self.CyA(x=roll, y=pitch, z=body_slip) * p_dyn
        Fz = self.CzA(x=roll, y=pitch, z=body_slip) * p_dyn

        Mx = self.MxA(x=roll, y=pitch, z=body_slip) * p_dyn
        My = self.MyA(x=roll, y=pitch, z=body_slip) * p_dyn
        Mz = self.MzA(x=roll, y=pitch, z=body_slip) * p_dyn

        CoPz = (Mz * Fz) / (Fx * Fy)
        CoPx = (CoPz * Fx - My) / Fz
        CoPy = (Mx + CoPz * Fy) / Fz

        return np.array([Fx, Fy, Fz, CoPx, CoPy, CoPz])