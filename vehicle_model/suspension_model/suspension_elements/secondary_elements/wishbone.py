from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
from vehicle_model.suspension_model.assets.misc_linalg import unit_vec
from vehicle_model.suspension_model.assets.plotter import Plotter
from typing import Sequence
import numpy as np


class Wishbone:
    """
    ## Wishbone

    Wishbone object

    Parameters
    ----------
    fore_link : Link
        Frontmost link of wishbone
    aft_link : Link
        Rearmost link of wishbone
    """
    def __init__(self, fore_link: Link, aft_link: Link) -> None:
        self.fore_link = fore_link
        self.aft_link = aft_link

        self.direction = self.direction_vec
        self.origin = Node(position=(fore_link.inboard_node.position + aft_link.inboard_node.position) / 2)
        self.angle = 0

        self.elements = [self.fore_link, self.aft_link]
        self.all_elements = [self.fore_link.inboard_node, self.fore_link.outboard_node, self.aft_link.inboard_node]

    def plot_elements(self, plotter) -> None:
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

    def rotate(self, angle: float) -> None:
        """
        ## Rotate

        Rotates wishbone about axis connecting inboard nodes

        Parameters
        ----------
        angle : float
            Angle of rotation in radians
        """
        self._set_initial_position()
        
        self.rot_mat = rotation_matrix(self.direction, angle)
        outboard_point = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position
        self.fore_link.outboard_node.position = np.matmul(self.rot_mat, outboard_point) + self.fore_link.inboard_node.position

        self.angle = angle
    
    def flatten_rotate(self, angle: Sequence[float]) -> None:
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
    
    def _set_initial_position(self) -> None:
        """
        ## Set Initial Position

        Resets position of wishbone to initial position

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.rot_mat = rotation_matrix(self.direction, -1 * self.angle)
        outboard_point = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position

        self.fore_link.outboard_node.position = np.matmul(self.rot_mat, outboard_point) + self.fore_link.inboard_node.position
    
    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates all children (inboard and outboard Nodes)

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply
            - Takes the form [x_shift, y_shift, z_shift]
        """
        for element in self.all_elements:
            element.translate(translation=translation)

    @property
    def plane(self) -> Sequence[float]:
        """
        ## Plane

        Calculates plane coincident with wishbone
        - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0

        Returns
        -------
        Sequence[float]
            Parameters defining plane: [a, b, c, x_0, y_0, z_0]
        """
        PQ = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position
        PR = self.aft_link.outboard_node.position - self.aft_link.inboard_node.position

        a, b, c = np.cross(PQ, PR)
        x_0, y_0, z_0 = self.fore_link.outboard_node.position

        return [a, b, c, x_0, y_0, z_0]

    @property
    def direction_vec(self) -> Sequence[float]:
        """
        ## Direction Vec

        Calculates unit vector from inboard aft node to inboard fore node

        Returns
        -------
        Sequence[float]
            Unit vector pointing from inboard aft node to inboard fore node
        """
        return unit_vec(self.fore_link.inboard_node, self.aft_link.inboard_node)