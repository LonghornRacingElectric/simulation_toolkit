from vehicle_model.suspension_model.suspension_elements.secondary_elements.kingpin import Kingpin
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class PushPullRod:
    """
    ## Push/Pull Rod

    Pushrod or pullrod element
    - Contains rod, bellcrank, and spring/damper mounting point

    Parameters
    ----------
    inboard : Node
        Node representing inboard end of push/pull rod
    outboard : Node
        Node representing outboard end of push/pull rod
    upper : bool
        True if rod mounts to upper wishbone
    bellcrank : bool
        Whether bellcrank is used
    bellcrank_pivot : Node, optional
        Point bellcrank rotates about, by default None
    bellcrank_direction : Sequence[float], optional
        Vector bellcrank rotates about, by default None
    shock_outboard : Node, optional
        Point where shock linkage meets bellcrank, by default None
    shock_inboard : Node, optional
        Point where shock linkage meets frame, by default None
    """
    def __init__(self, inboard: Node, outboard: Node, upper: bool, bellcrank: bool, bellcrank_pivot: Node | None = None, 
                 bellcrank_direction: Sequence[float] | None = None, shock_outboard: Node | None = None, 
                 shock_inboard: Node | None = None) -> None:
        
        self.rod = Link(inboard=inboard, outboard=outboard)
        self.initial_rod_length = self.rod_length

        self.bellcrank_angle = 0
        self.wishbone_angle = 0

        if bellcrank:
            self.bellcrank_pivot = bellcrank_pivot
            self.bellcrank_direction = bellcrank_direction
            self.spring_damper_rod = Link(inboard=shock_inboard, outboard=shock_outboard)
            self.initial_spring_damper_length = self.spring_damper_length

        if bellcrank:
            self.elements = [self.rod, self.bellcrank_pivot, self.spring_damper_rod]
            self.all_elements = [self.rod, self.bellcrank_pivot, self.spring_damper_rod]
        else:
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
        self._set_initial_position()
        self.rot_mat = rotation_matrix(axis, angle)
        translated_point = self.rod.outboard_node.position - origin.position
        self.rod.outboard_node.position = np.matmul(self.rot_mat, translated_point) + origin.position

    def rotate_bellcrank(self, angle: float) -> None:
        """
        ## Rotate Bellcrank

        Rotates bellcrank and its respective links

        Parameters
        ----------
        angle : float
            Angle of bellcrank rotation in radians
        """
        bellcrank_rot_mat = rotation_matrix(unit_vec=self.bellcrank_direction, theta=angle)
        # Rotate rod about bellcrank
        rod_rotated_point = self.rod.inboard_node.position - self.bellcrank_pivot.position
        self.rod.inboard_node.position = np.matmul(bellcrank_rot_mat, rod_rotated_point) + self.bellcrank_pivot.position

        # Rotate spring/damper about bellcrank
        spring_damper_rotated_point = self.spring_damper_rod.outboard_node.position - self.bellcrank_pivot.position
        self.spring_damper_rod.outboard_node.position = np.matmul(bellcrank_rot_mat, spring_damper_rotated_point) + self.bellcrank_pivot.position

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

    @property
    def spring_damper_length(self) -> float:
        """
        ## Spring/Damper Length

        Length of spring/damper linkage

        Returns
        -------
        float
            Length of spring/damper linkage
        """
        return np.linalg.norm(self.spring_damper_rod.inboard_node.position - self.spring_damper_rod.outboard_node.position)
    
    def update(self) -> None:
        """
        ## Update

        Updates rod outboard position based on jounce condition
        - Rotated about bellcrank

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        # Recalculate steering pickup based on jounce condition
        bellcrank_angle = fsolve(self._bellcrank_resid_func, [0])
        
        self.rotate_bellcrank(angle=bellcrank_angle[0])
    
    def _bellcrank_resid_func(self, x: Sequence[float]) -> Sequence[float]:
        """
        ## Bellcrank Residual Function

        Bellcrank residual function for bellcrank angle convergence

        Parameters
        ----------
        angle : Sequence[float]
            Angle of bellcrank rotation in radians
        
        Returns
        -------
        Sequence[float]
            Residuals
        """
        bellcrank_rotation = x[0]
        self.bellcrank_angle = bellcrank_rotation
        self.rotate_bellcrank(angle=bellcrank_rotation)

        residual = [self.initial_rod_length - self.rod_length]
        self._reset_bellcrank_position()

        return residual
    
    def _reset_bellcrank_position(self) -> None:
        """
        ## Reset Bellcrank Position

        Resets angle of bellcrank alone

        Parameters
        ----------
        None
        """
        # Rod inboard node reset
        self.rod.inboard_node.reset()

        # Shock outboard node reset
        self.spring_damper_rod.outboard_node.reset()
    
    def _set_initial_position(self) -> None:
        """
        ## Set Initial Position

        Resets position of bellcrank and respective linkages

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        # Rod outboard node reset
        self.rod.outboard_node.reset()

        self._reset_bellcrank_position()
    
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