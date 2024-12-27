from typing import Sequence, Tuple, Union, Callable
import numpy as np


def unit_vec(p1: Union[np.ndarray, Sequence[float]], p2: Union[np.ndarray, Sequence[float]]) -> Sequence[float]:
    """
    ## Unit Vector Calculation

    Calculates unit vector given two points

    Parameters
    ----------
    p1 : Node
        Starting point
    p2 : Node
        Ending point

    Returns
    -------
    np.ndarray
        Unit vector pointing from starting node to ending node
    """
    p1 = np.array(p1)
    p2 = np.array(p2)
    vector_AB = p2 - p1
    vector_AB_mag = np.linalg.norm(p2 - p1)
    
    return [float(x) for x in vector_AB / vector_AB_mag]

def rotation_matrix(unit_vec: Union[np.ndarray, Sequence[float], Tuple[float, float, float]], theta: float) -> Sequence[Sequence[float]]:
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

def nearest_root(func: Callable, x0: float, bounds: Tuple[float, float], tol: float, args: Sequence = []):
    """
    ## Nearest Root

    Finds the root nearerst an initial guess

    Parameters
    ----------
    func : Callable
        Optimization function. Follows the form: func(x: float, args: Sequence)

    x0 : float
        Initial guess

    bounds : Tuple[float, float]
        Bounds on solution inputs

    tol : float
        Tolerance on solution

    args : Sequence, optional
        Args to func, by default []
    """
    # Optimization parameters
    residual: float = 1e9
    previous_residual: float = 1e9

    # Check direction to iterate in
    positive: bool = True

    positive_check: float = abs(func(x0+tol, args))
    negative_check: float = abs(func(x0-tol, args))

    if positive_check > negative_check:
        positive = False

    # Solution value
    soln = x0

    # Steps
    step_size = (bounds[1] - bounds[0]) / 10

    while abs(residual) > tol:
        previous_residual = residual

        if positive:
            soln += step_size
        else:
            soln -= step_size

        residual = func(soln, args)

        if abs(residual) < abs(previous_residual):
            continue
        else:
            step_size /= 2
            positive = not positive
    
    return soln