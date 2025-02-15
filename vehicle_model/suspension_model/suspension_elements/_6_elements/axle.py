from vehicle_model.suspension_model.suspension_elements._4_elements.push_pull_rod import PushPullRod
from vehicle_model.suspension_model.suspension_elements._5_elements.quarter_car import QuarterCar
from vehicle_model.suspension_model.suspension_elements._2_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements._2_elements.stabar import Stabar
from vehicle_model.suspension_model.suspension_elements._2_elements.tire import Tire
from vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from vehicle_model.assets.misc_math import unit_vec, rotation_matrix

# from scipy.interpolate import CubicSpline
from typing import Sequence, Tuple, Union
from scipy.optimize import fsolve # type: ignore
import numpy as np


class Axle:
    """
    ## Axle

    Two full corner assemblies
    - Includes two quarter cars and a stabar if applicable

    Parameters
    ----------
    quarter_car_left : QuarterCar
        Left quarter car assembly
    
    quarter_car_right : QuarterCar
        Right quarter car assembly

    front_stabar : Stabar
        Front stabar

    rear_stabar : Stabar
        Rear stabar
    """
    def __init__(
            self,
            quarter_car_left: QuarterCar,
            quarter_car_right: QuarterCar,
            front_stabar: Union[Stabar, None],
            rear_stabar: Union[Stabar, None]):
        
        pass
    