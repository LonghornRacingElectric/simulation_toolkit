from vehicle_model.suspension_model.suspension_elements.secondary_elements.kingpin import Kingpin
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class DirectRod:
    """
    ## Direct Acting Rod

    Pushrod or pullrod element
    - Contains push/pullrod and spring/damper

    Parameters
    ----------
    inboard : Node
        Node representing inboard end of push/pull rod
    outboard : Node
        Node representing outboard end of push/pull rod
    upper : bool
        True if rod mounts to upper wishbone
    """
    def __init__(self, inboard: Node, outboard: Node) -> None:
        
        self.rod = Link(inboard=inboard, outboard=outboard)
        self.initial_rod_length = self.rod_length

        self.elements = [self.rod]
        self.all_elements = [self.rod]

    def rotate_rod(self, axis: Sequence[float], origin: Node, angle: float) -> None:
        """
        ## Rotate Rod

        Rotates outboard push/pull rod node about given axis

        Parameters
        ----------
        axis : Sequence[float]
            Unit vector giving direction of rotation axis
        origin : Node
            Origin of transformation
        angle : float
            Angle of rotation in radians

        Returns
        -------
        None
        """
        self.rod.inboard_node.reset()
        self.rod.outboard_node.reset()
        self.rot_mat = rotation_matrix(axis, angle)
        translated_point = self.rod.outboard_node.position - origin.position
        self.rod.outboard_node.position = np.matmul(self.rot_mat, translated_point) + origin.position

    @property
    def rod_length(self) -> float:
        """
        ## Rod Length

        Length of rod

        Returns
        -------
        float
            Length of rod
        """
        return np.linalg.norm(self.rod.inboard_node.position - self.rod.outboard_node.position)
    
    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates all children

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply
            - Takes the form [x_shift, y_shift, z_shift]
        """
        for element in self.all_elements:
            element.translate(translation=translation)
    
    def flatten_rotate(self, angle: Sequence[float]):
        """
        ## Flatten Rotate

        Rotates all children
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        angle : Sequence[float]
            Angles of rotation in radians [x_rot, y_rot, z_rot]
        """
        for element in self.all_elements:
            element.flatten_rotate(angle=angle)

    def plot_elements(self, plotter):
        """
        ## Plot Elements

        Plots all child elements

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        for element in self.elements:
            element.plot_elements(plotter=plotter)