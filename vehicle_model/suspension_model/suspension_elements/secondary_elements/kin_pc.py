from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from typing import Sequence


class KinPC:
    """
    ## KinPC

    Kinematic Pitch Center

    Parameters
    ----------
    front_swing_arm : Link
        Link from front contact patch to the corner's side-view instance center
    rear_swing_arm : Link
        Link from rear contact patch to the corner's side-view instance center
    """
    def __init__(self, front_swing_arm: Link, rear_swing_arm: Link, cg: CG) -> None:
        self.front_swing_arm = front_swing_arm
        self.rear_swing_arm = rear_swing_arm
        self.cg = cg

        # Actual kinematic roll center
        self.true_KinPC = Node(position=self.true_KinPC_pos)
        self.long_position, self.vertical_position = self.true_KinPC.position[0], self.true_KinPC.position[2]

        self.cg_axis_KinPC = Node(position=self.cg_axis_KinPC_pos)
        
        # self.elements = [self.true_KinPC, self.cg_axis_KinPC]
        self.elements = [self.true_KinPC]
        # self.all_elements = [self.true_KinPC, self.cg_axis_KinPC]
        self.all_elements = [self.true_KinPC]

    def update(self) -> None:
        """
        ## Update

        Updates position of pitch center

        Parameters
        -------
        None

        Returns
        -------
        None
        """
        self.true_KinPC.position = self.true_KinPC_pos
        self.long_position, self.vertical_position = self.true_KinPC.position[0], self.true_KinPC.position[2]
        self.cg_axis_KinPC.position = self.cg_axis_KinPC_pos

    @property
    def cg_axis_KinPC_pos(self) -> Sequence[float]:
        """
        ## CG-Axis KinPC Position

        Position of kinematic pitch center when projected about vertical axis of sprung mass

        Returns
        -------
        Sequence[float]
            Coordinates of transformed pitch center
        """
        x = (self.true_KinPC.position[2] - self.cg.position[2]) * self.cg.direction[0] / self.cg.direction[2] + self.cg.position[0]
        return [x, self.true_KinPC.position[1], self.true_KinPC.position[2]]

    @property
    def true_KinPC_pos(self) -> Sequence[float]:
        """
        ## True KinPC Position

        True position of kinematic pitch center

        Returns
        -------
        Sequence[float]
            Coordinates of true pitch center
        """
        return self.front_swing_arm.xz_intersection(link=self.rear_swing_arm)
    
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