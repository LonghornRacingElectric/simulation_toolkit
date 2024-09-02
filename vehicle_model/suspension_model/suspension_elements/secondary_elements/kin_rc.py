from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from typing import Sequence


class KinRC:
    """
    ## KinRC

    Kinematic Roll Center

    Parameters
    ----------
    left_swing_arm : Link
        Link from left contact patch to the corner's front-view instance center
    right_swing_arm : Link
        Link from right contact patch to the corner's front-view instance center
    """
    def __init__(self, left_swing_arm: Link, right_swing_arm: Link, cg: CG) -> None:
        self.left_swing_arm = left_swing_arm
        self.right_swing_arm = right_swing_arm
        self.cg = cg

        # Actual kinematic roll center
        self.true_KinRC = Node(position=self.true_KinRC_pos)
        self.lateral_position, self.vertical_position = self.true_KinRC.position[1:]

        self.cg_axis_KinRC = Node(position=self.cg_axis_KinRC_pos)
        
        # self.elements = [self.true_KinRC, self.cg_axis_KinRC]
        self.elements = [self.true_KinRC]
        # self.all_elements = [self.true_KinRC, self.cg_axis_KinRC]
        self.all_elements = [self.true_KinRC]

    def update(self) -> None:
        """
        ## Update

        Updates position of roll center

        Parameters
        -------
        None

        Returns
        -------
        None
        """
        self.true_KinRC.position = self.true_KinRC_pos
        self.lateral_position, self.vertical_position = self.true_KinRC.position[1:]
        self.cg_axis_KinRC.position = self.cg_axis_KinRC_pos

    @property
    def cg_axis_KinRC_pos(self) -> Sequence[float]:
        """
        ## CG-Axis KinRC Position

        Position of kinematic roll center when projected about vertical axis of sprung mass

        Returns
        -------
        Sequence[float]
            Coordinates of transformed roll center
        """
        y = (self.true_KinRC.position[2] - self.cg.position[2]) * self.cg.direction[1] / self.cg.direction[2] + self.cg.position[1]
        return [self.true_KinRC.position[0], y, self.true_KinRC.position[2]]

    @property
    def true_KinRC_pos(self) -> Sequence[float]:
        """
        ## True KinRC Position

        True position of kinematic roll center

        Returns
        -------
        Sequence[float]
            Coordinates of true roll center
        """
        return self.left_swing_arm.yz_intersection(link=self.right_swing_arm)
    
    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates all children

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply

        Returns
        ----------
        None
        """
        for element in self.all_elements:
            element.translate(translation=translation)
    
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

    def plot_elements(self, plotter) -> None:
        """
        ## Plot Elements

        Plots all children

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        for element in self.elements:
            element.plot_elements(plotter=plotter)