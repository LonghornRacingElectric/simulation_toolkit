from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.assets.misc_linalg import rotation_matrix
from typing import Sequence, Tuple, Union
import numpy as np


class Bellcrank:
    """
    ## Bellcrank

    Bellcrank object

    Parameters
    ----------
    *nodes : Sequence[Node]
        All distinct pickup Nodes on bellcrank

    pivot : Node
        Bellcrank pivot Node

    pivot_direction : Tuple[Union[float, int], Union[float, int], Union[float, int]]
        Unit vector representing Bellcrank pivot axis
    """
    def __init__(self, *nodes: Node, pivot: Node, pivot_direction: Tuple[Union[float, int], Union[float, int], Union[float, int]]) -> None:
        self.nodes: Sequence[Node] = nodes
        self.pivot = pivot
        self.pivot_direction = pivot_direction

        self.angle: Union[float, int] = 0

    def rotate(self, angle: Union[float, int]) -> None:
        """
        ## Rotate

        Rotates bellcrank

        Parameters
        ----------
        angle : Union[float, int]
            Angle of rotation in radians
        """
        for node in self.nodes:
            node.reset()

            node_translated = node - self.pivot
            rot = rotation_matrix(unit_vec=self.pivot_direction, theta=angle)

            node.position = list(np.matmul(rot, node_translated.position) + np.array(self.pivot.position))
        
        self.angle = angle