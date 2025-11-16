from src.vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from src._3_custom_libraries.misc_math import nearest_root

from typing import Tuple
import numpy as np


class Stabar:
    """
    ## Stabar

    Stabar object

    Parameters
    ----------
    left_arm_end : Node
        End (farthest radially from the axis of the torsion bar) of stabar lever in positive y

    right_arm_end : Node
        End (farthest radially from the axis of the torsion bar) of stabar lever in negative y

    left_droplink_end : Node
        Droplink mounting point to rest of car in positive y

    right_droplink_end : Node
        Droplink mounting point to rest of car in negative y

    bar_left_end : Node
        End of torsion bar in positive y
    
    bar_right_end : Node
        End of torsion bar in negative y

    torsional_stiffness : float
        Torsional stiffness of entire stabar
    """
    def __init__(self, 
                 left_arm_end: Node, 
                 right_arm_end: Node, 
                 left_droplink_end: Node, 
                 right_droplink_end: Node, 
                 bar_left_end: Node, 
                 bar_right_end: Node, 
                 torsional_stiffness: float) -> None:

        # Torsion bar
        self.bar: Link = Link(inboard_node=bar_left_end, outboard_node=bar_right_end)
        self.torsional_stiffness: float = torsional_stiffness

        # Arms
        self.left_arm: Link = Link(inboard_node=bar_left_end, outboard_node=left_arm_end)
        self.right_arm: Link = Link(inboard_node=bar_right_end, outboard_node=right_arm_end)

        # Droplinks
        self.left_droplink: Link = Link(inboard_node=left_droplink_end, outboard_node=left_arm_end)
        self.right_droplink: Link = Link(inboard_node=right_droplink_end, outboard_node=right_arm_end)

        # Rotations for tracking
        self.left_rotation: float = 0
        self.right_rotation: float = 0

    def update(self) -> None:
        """
        ## Update

        Updates Stabar to match initial geometry

        """
        self.left_rotation = nearest_root(func=self._droplink_eqn, x0=0, bounds=(-np.pi/2, np.pi/2), tol=1e-10, args=[self.left_droplink])
        self.right_rotation = nearest_root(func=self._droplink_eqn, x0=0, bounds=(-np.pi/2, np.pi/2), tol=1e-10, args=[self.right_droplink])

    def _droplink_eqn(self, x: float, args: Tuple[Link]) -> float:
        """
        ## Droplink Equation

        Residual function for droplink length convergence

        Parameters
        ----------
        x : float
            Angle of rotation in radians

        args : Tuple[Link]
            Arguments in the form: [Droplink Link]

        Returns
        -------
        float
            Convergence criteria
        """
        rotation=x
        droplink = args[0]

        droplink.outboard_node.reset()

        droplink.outboard_node.rotate(origin=self.bar.inboard_node, direction=self.bar.direction, angle=rotation)

        return droplink.length - droplink.initial_length

    @property
    def rotation(self) -> float:
        """
        ## Rotation

        Angualar deformation of stabar

        Returns
        -------
        float
            Angular deformation of stabar
        """
        return self.left_rotation - self.right_rotation
    
    @property
    def torque(self) -> float:
        """
        ## Torque

        Torque reacted by stabar

        Returns
        -------
        float
            Torque reacted by stabar
        """
        return abs(self.torsional_stiffness * self.rotation)