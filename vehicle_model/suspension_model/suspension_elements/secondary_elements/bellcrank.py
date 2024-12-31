from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.assets.misc_math import rotation_matrix
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

        - First entry should connect to push/pull rod
        - Final entry should connect to inboard rod/shock

    pivot : Node
        Bellcrank pivot Node

    pivot_direction : Tuple[float, float, float]
        Unit vector representing Bellcrank pivot axis
    """
    def __init__(self, *nodes: Node, pivot: Node, pivot_direction: Tuple[float, float, float]) -> None:
        self.nodes: Sequence[Node] = nodes
        self.pivot = pivot
        self.pivot_direction = pivot_direction

        self.angle: float = 0

    def rotate(self, angle: float) -> None:
        """
        ## Rotate

        Rotates bellcrank

        Parameters
        ----------
        angle : float
            Angle of rotation in radians
        """
        for node in self.nodes:
            node.reset()
            node.rotate(origin=self.pivot, direction=self.pivot_direction, angle=angle)
        
        self.angle = angle