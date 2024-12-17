from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.assets.misc_linalg import rotation_matrix, unit_vec
from typing import Sequence, Union
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
        self.fore_link: Link = fore_link
        self.aft_link: Link = aft_link

        self.direction: np.ndarray = unit_vec(p1=self.fore_link.inboard_node.position, p2=self.aft_link.inboard_node.position)
        self.angle: Union[float, int] = 0.0

    def rotate(self, angle: float) -> None:
        """
        ## Rotate

        Rotates wishbone about axis connecting inboard nodes

        Parameters
        ----------
        angle : float
            Angle of rotation in radians
        """
        net_angle = self.angle + angle
        self.rot_mat = rotation_matrix(self.direction, net_angle)
        self.angle = net_angle

        outboard_point = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position
        self.fore_link.outboard_node.position = np.matmul(self.rot_mat, outboard_point) + self.fore_link.inboard_node.position

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
    def direction_vec(self) -> np.ndarray:
        """
        ## Direction Vec

        Calculates unit vector from inboard aft node to inboard fore node

        Returns
        -------
        Sequence[float]
            Unit vector pointing from inboard aft node to inboard fore node
        """
        return unit_vec(p1=self.fore_link.inboard_node.position, p2=self.aft_link.inboard_node.position)