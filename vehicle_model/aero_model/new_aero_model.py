from scipy.interpolate import LinearNDInterpolator
from typing import Callable, Sequence, Tuple
import pandas as pd
import numpy as np


class AeroModel:
    """
    
    """
    def __init__(self, csv_path: str, air_density: float) -> None:
            
        self.parameters = locals().copy()
        del self.parameters['self']

        self.forces = [0,0,0,0]
        self.air_density = air_density
        self.aero_map = pd.read_csv(csv_path)

        self.roll_array = self.aero_map['Roll'].to_numpy()
        self.pitch_array = self.aero_map['Pitch'].to_numpy()
        self.yaw_array = self.aero_map['Yaw'].to_numpy()

    def eval(self, roll: float, pitch: float, body_slip: float, heave: float, velocity: float):
        """
        ## Aero Evaluation

        Calculate the forces from aerodynamic effects

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

        [F_x, F_y, F_z], [M_x,M_y,M_z] = self._get_loads(velocity, body_slip * 180 / np.pi, heave * 0.0254, pitch * 180 / np.pi, roll * 180 / np.pi)

        # if F_z != 0 and F_x != 0 and F_y != 0:
            # Solve for z_CoP
        z_CoP = -(M_z * F_z) / (F_x * F_y)
        
        # Solve for x_CoP and y_CoP using the z_CoP result
        x_CoP = (z_CoP * F_x - M_y) / F_z
        y_CoP = (M_x + z_CoP * F_y) / F_z
        # else:
        #     x_CoP, y_CoP, z_CoP = np.inf, np.inf, np.inf  # Handle division by zero

        return np.array([F_x, F_y, F_z, x_CoP * 0.0254, y_CoP * 0.0254, z_CoP * 0.0254])

    def _get_loads(self, speed: float, body_slip: float, heave: float, pitch: float, roll: float):

        if roll > self.roll_array.max():
            roll = self.roll_array.max() - 0.0001
        if roll < self.roll_array.min():
            roll = self.roll_array.min() + 0.0001
        if pitch > self.pitch_array.max():
            pitch = self.pitch_array.max() - 0.0001
        if pitch < self.pitch_array.min():
            pitch = self.pitch_array.min() + 0.0001
        if body_slip > self.yaw_array.max():
            body_slip = self.yaw_array.max() - 0.0001
        if body_slip < self.yaw_array.min():
            body_slip = self.yaw_array.min() + 0.0001
        
        coeffs = ['cx','cy','cz','mx','my','mz']
        results = np.zeros(len(coeffs))
        points = np.array([self.roll_array, self.pitch_array, self.yaw_array]).T  # Shape should be (n, 3) where n is the number of samples
        for i,coeff in enumerate(coeffs):
            # Create the interpolator object
            interpolator = LinearNDInterpolator(points, self.aero_map[coeff].to_numpy(), fill_value=0.00001)
            # Now you can use this interpolator to find cx for new pitch, yaw, roll values
            new_pitch = pitch
            new_yaw = body_slip
            new_roll = roll
            results[i] = interpolator(new_roll, new_pitch, new_yaw)
        return results[0:3]*self.air_density*0.5*speed**2, results[3:]*self.air_density*0.5*speed**2

