from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
from vehicle_model.suspension_model.suspension_elements.secondary_elements.kingpin import Kingpin
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from typing import Sequence
import numpy as np


class Tire:
    """
    ## Tire

    Tire object

    Parameters
    ----------
    contact_patch : Node
        Contact patch
    kingpin : Kingpin
        Kingpin
    static_gamma : float
        Static inclination angle in radians
    static_toe : float
        Static toe angle in radians
    radius : float
        Tire radius
    width : float
        Tire width
    """
    def __init__(self, 
                 contact_patch: Node, 
                 kingpin: Kingpin,  
                 static_gamma: float,
                 static_toe: float,
                 radius: float, 
                 width: float) -> None:
        
        self.cp: Node = contact_patch
        self.kingpin = kingpin
        self.gamma: float = static_gamma
        self.radius = radius
        self.width = width
        self.static_toe = static_toe
        self._induced_steer = static_toe

        # Calculate initial center
        rotation = rotation_matrix([1, 0, 0], self.gamma)
        self.initial_center = np.matmul(rotation, [0, 0, 1]) * self.radius + self.cp.position

        # Calculate center relative to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        center_shifted = self.initial_center - self.kingpin.inboard_node.position
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.center_to_kingpin = np.matmul(y_rot, np.matmul(x_rot, center_shifted))

        # Calculate contact patch relative to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        cp_shifted = self.cp.initial_position - self.kingpin.inboard_node.position
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.cp_to_kingpin = np.matmul(y_rot, np.matmul(x_rot, cp_shifted))

        # Calculate static direction
        rotation = rotation_matrix([1, 0, 0], self.gamma)
        self.initial_kpi, self.initial_caster = self.kingpin.normalized_transform()
        self.initial_direction = np.matmul(rotation, [0, 1, 0])
        
        # Calculate direction relative to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.tire_direction = np.matmul(y_rot, np.matmul(x_rot, self.initial_direction))

        self.elements = [self.cp]
        self.all_elements = [self.cp]
    
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

        Plots all child elements

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        plotter.add_tire(center=self.center, direction=self.direction, radius=self.radius, height=self.height)
        for element in self.elements:
            element.plot_elements(plotter=plotter)
    
    @property
    def direction(self) -> Sequence[float]:
        """
        ## Direction

        Direction of tire attribute (parallel to tire y-axis at no alpha or gamma)

        Returns
        -------
        Sequence[float]
            Unit vector of tire direction
        """
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.induced_steer)
        direction = np.matmul(y_rot, np.matmul(x_rot, np.matmul(z_rot, self.tire_direction)))

        return direction

    @property
    def center(self) -> Sequence[float]:
        """
        ## Center

        Center of tire attribute

        Returns
        -------
        Sequence[float]
            Coordinates of tire center
        """
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.induced_steer)
        center_position = np.matmul(y_rot, np.matmul(x_rot, np.matmul(z_rot, self.center_to_kingpin))) + self.kingpin.inboard_node.position

        return center_position

    @property
    def height(self) -> float:
        """
        ## Height

        Width of tire attribute

        Returns
        -------
        float
            Width of tire
        """
        return self.width
    
    @property
    def induced_steer(self) -> float:
        """
        ## Induced Steer

        Effective steer angle at tire

        Returns
        -------
        float
            Effective steer angle at tire in radians
        """
        return self._induced_steer
    
    @induced_steer.setter
    def induced_steer(self, value: float) -> None:
        """
        ## Induced Steer Setter

        Sets induced steer of tire

        Parameters
        ----------
        value : float
            Induced steer angle in radians
        """
        self._induced_steer = self.static_toe
        self._induced_steer += value

        # Update contact patch position
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self._induced_steer)
        self.cp.position = np.matmul(x_rot, np.matmul(y_rot, np.matmul(z_rot, self.cp_to_kingpin))) + self.kingpin.inboard_node.position