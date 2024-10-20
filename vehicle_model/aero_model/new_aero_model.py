
# from sim.model_parameters.cars.car import Car
# from sim.system_models.vectors.controls_vector import ControlsVector
# from sim.system_models.vectors.observables_vector import ObservablesVector
# from sim.system_models.vectors.state_vector import StateVector
# from sim.system_models.vectors.state_dot_vector import StateDotVector
from vehicle_model.suspension_model.assets.interp import interp3d

from scipy.interpolate import LinearNDInterpolator

import numpy as np
import pandas as pd

from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_elements.quinary_elements.full_suspension import FullSuspension
from vehicle_model.suspension_model.suspension_elements.quaternary_elements.axle import Axle
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from vehicle_model.suspension_model.assets.interp import interp4d
from vehicle_model._assets.pickle_helpers import pickle_import
from matplotlib.backends.backend_pdf import PdfPages
from typing import Callable, Sequence, Tuple
from matplotlib.figure import Figure
from collections import OrderedDict
from dash import Dash, dash_table
import matplotlib.pyplot as plt
import pyvista as pv
import pandas as pd
import numpy as np
import pickle
import os


class AeroModel:
    """
    
    """
    def __init__(
            self,
            csv_path: str,
            air_density: float
            ) -> None:
            
        self.parameters = locals().copy()
        del self.parameters['self']

        self.forces = [0,0,0,0]
        self.air_density = air_density
        self.aero_map = pd.read_csv(csv_path)

    def eval(self,
            roll: float,
            pitch: float,
            body_slip: float,
            heave: float,
            velocity: float):

        [F_x, F_y, F_z], [M_x,M_y,M_z] = self._get_loads(velocity, body_slip, heave, pitch, roll)
        


        if F_z != 0 and F_x != 0 and F_y != 0:
            # Solve for z_CoP
            z_CoP = -(M_z * F_z) / (F_x * F_y)
            
            # Solve for x_CoP and y_CoP using the z_CoP result
            x_CoP = (z_CoP * F_x - M_y) / F_z
            y_CoP = (M_x + z_CoP * F_y) / F_z
        else:
            x_CoP, y_CoP, z_CoP = np.inf, np.inf, np.inf  # Handle division by zero

        return np.array([F_x, F_y, F_z]), np.array([x_CoP, y_CoP, z_CoP])



    def _get_loads(self, speed: float, body_slip: float, heave: float, pitch: float, roll: float):
        
        coeffs = ['cx','cy','cz','mx','my','mz']
        results = np.zeros(len(coeffs))
        points = np.array([self.aero_map['Roll'].to_numpy(),self.aero_map['Pitch'].to_numpy(),self.aero_map['Yaw'].to_numpy()]).T  # Shape should be (n, 3) where n is the number of samples
        for i,coeff in enumerate(coeffs):
            # Create the interpolator object
            interpolator = LinearNDInterpolator(points, self.aero_map[coeff].to_numpy())
            # Now you can use this interpolator to find cx for new pitch, yaw, roll values
            new_pitch = pitch
            new_yaw = body_slip
            new_roll = roll
            results[i] = interpolator(new_roll, new_yaw, new_pitch)
        return results[0:3]*self.air_density*0.5*speed**2, results[3:]*self.air_density*0.5*speed**2

