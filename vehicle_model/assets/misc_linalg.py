from typing import Sequence
import numpy as np


def unit_vec(p1: Sequence[float], p2: Sequence[float]) -> np.ndarray:
    """
    ## Unit Vector Calculation

    Calculates unit vector given two points

    Parameters
    ----------
    p1 : Node
        Ending point
    p2 : Node
        Starting point

    Returns
    -------
    np.ndarray
        Unit vector pointing from starting node to ending node
    """
    p1 = np.array(p1)
    p2 = np.array(p2)
    vector_AB = p1 - p2
    vector_AB_mag = np.linalg.norm(p1 - p2)
    
    return vector_AB / vector_AB_mag

def rotation_matrix(unit_vec: np.array, theta: float) -> Sequence[Sequence[float]]:
    """
    ## Rotation Matrix

    Generates rotation matrix

    Parameters
    ----------
    unit_vec : np.array
        Unit vector along which to perform rotation
    theta : float
        Angle for desired rotation (in radians)

    Returns
    -------
    Sequence[Sequence[float]]
        Rotation matrix for desired angle about desired axis
    """
    ux = unit_vec[0]
    uy = unit_vec[1]
    uz = unit_vec[2]
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    matrix = [
        [ux**2 * (1 - cos_t) + cos_t, 
         ux * uy * (1 - cos_t) - uz * sin_t, 
         ux * uz * (1 - cos_t) + uy * sin_t], 
        
        [ux * uy * (1 - cos_t) + uz * sin_t,
         uy**2 * (1 - cos_t) + cos_t,
         uy * uz * (1 - cos_t) - ux * sin_t], 
        
        [ux * uz * (1 - cos_t) - uy * sin_t,
         uy * uz * (1 - cos_t) + ux * sin_t,
         uz**2 * (1 - cos_t) + cos_t]
        ]
    
    return matrix