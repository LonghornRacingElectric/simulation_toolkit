import vehicle_model.assets.misc_linalg as linalg
from typing import Sequence
import numpy as np


class Node:
    """
    ## Node

    Node object
    - Similar to node element

    Parameters
    ----------
    Position : Sequence[float]
        Position of Node
    """
    def __init__(self, position: Sequence[float]) -> None:
        self.position = np.array(position)
        self.initial_position = np.array(position)

    def reset(self) -> None:
        """
        ## Reset

        Resets Node position to initial position

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.position = self.initial_position
    
    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates Node

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply

        Returns
        ----------
        None
        """
        self.position = self.position + np.array(translation)
    
    def rotate(self, origin: "Node", angle_x: float = 0, angle_y: float = 0, angle_z: float = 0) -> None:
        """
        ## Flatten Rotate

        Rotates Node
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        origin : Node
            Node representing origin of transformation
        angle_x : float
            Angle of rotation about x in radians
        angle_y : float
            Angle of rotation about y in radians
        angle_z : float
            Angle of rotation about z in radians
        """
        x_rot = linalg.rotation_matrix(unit_vec=[1, 0, 0], theta=angle_x)
        y_rot = linalg.rotation_matrix(unit_vec=[0, 1, 0], theta=angle_y)
        z_rot = linalg.rotation_matrix(unit_vec=[0, 0, 1], theta=angle_z)
        self.position = np.matmul(z_rot, np.matmul(y_rot, np.matmul(x_rot, self.position - origin.position))) + origin.position