import vehicle_model.suspension_model.assets.misc_linalg as linalg
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
        self.initial_position = position

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
    
    def flatten_rotate(self, angle: Sequence[float]) -> None:
        """
        ## Flatten Rotate

        Rotates Node
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        angle : Sequence[float]
            Angles of rotation in radians [x_rot, y_rot, z_rot]
        """
        x_rot = linalg.rotation_matrix(unit_vec=[1, 0, 0], theta=angle[0])
        y_rot = linalg.rotation_matrix(unit_vec=[0, 1, 0], theta=angle[1])
        z_rot = linalg.rotation_matrix(unit_vec=[0, 0, 1], theta=angle[2])
        self.position = np.matmul(x_rot, np.matmul(y_rot, np.matmul(z_rot, self.position)))

    def plot_elements(self, plotter, radius: float = 0.875 / 2) -> None:
        """
        ## Plot Elements

        Plots Node

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        radius: float
            Radius of Node
        """
        if max(np.abs(self.position)) < 1000:
            plotter.add_node(center=self.position, radius=radius)