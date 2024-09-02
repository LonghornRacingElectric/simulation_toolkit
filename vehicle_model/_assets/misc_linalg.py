from typing import Sequence
import numpy as np

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

def plane(points: Sequence[Sequence[float]]) -> Sequence[float]:
    """
    ## Plane

    Calculates plane from a collection of three points
    - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0

    Parameters
    ----------
    points : Sequence[Sequence[float]]
        Three points defining plane

    Returns
    -------
    Sequence[float]
        Parameters defining plane: [a, b, c, x_0, y_0, z_0]
    """
    assert len(points) == 3, f"Plane generator only accepts 3 points | {len(points)} points were given"
    PQ = points[1] - points[0]
    PR = points[2] - points[0]

    a, b, c = np.cross(PQ, PR)
    x_0, y_0, z_0 = points[0]

    return [a, b, c, x_0, y_0, z_0]

def plane_eval(points: Sequence[Sequence[float]], x: float, y: float) -> Sequence[float]:
    """
    ## Plane Eval

    Calculates point on a plane given two independent variables
    - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0

    Parameters
    ----------
    points : Sequence[Sequence[float]]
        Three points defining plane

    Returns
    -------
    Sequence[float]
        Point on plane: [x, y, z]
    """
    assert len(points) == 3, f"Plane generator only accepts 3 points | {len(points)} points were given"
    PQ = points[1] - points[0]
    PR = points[2] - points[0]

    a, b, c = np.cross(PQ, PR)
    x_0, y_0, z_0 = points[0]

    z = a / c *(x_0 - x) + b / c * (y_0 - y) + z_0

    return [x, y, z]