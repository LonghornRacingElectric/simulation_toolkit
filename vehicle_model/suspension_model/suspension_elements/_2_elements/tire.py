from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from _4_custom_libraries.misc_math import rotation_matrix

from LHR_tire_toolkit.MF52 import MF52 # type: ignore
import numpy as np

class Tire:
    def __init__(self, 
                 tire: MF52, 
                 contact_patch: Node,
                 outer_diameter: float, 
                 width: float, 
                 inner_diameter: float,
                 static_toe: float = 0,
                 static_gamma: float = 0) -> None:
        
        self.tire = tire
        self.contact_patch = contact_patch
        self.outer_diameter = outer_diameter
        self.width = width
        self.inner_diameter = inner_diameter
        self.steered_angle: float = 0.0

        self.static_toe: float = static_toe
        self.static_gamma: float = static_gamma

        # This only works for Z-up SAE J670 coords
        x_rot = np.array(rotation_matrix(unit_vec=[1, 0, 0], theta=static_gamma * np.pi / 180))
        z_rot = np.array(rotation_matrix(unit_vec=[0, 0, 1], theta=static_toe * np.pi / 180))
        
        center_vec = np.array([0, 0, outer_diameter / 2])
        front_vec = np.array([outer_diameter / 2, 0, outer_diameter / 2])

        self.center_node = Node(position=np.array(contact_patch.position) + z_rot @ x_rot @ center_vec)
        self.front_node = Node(position=np.array(contact_patch.position) + z_rot @ x_rot @ front_vec)
        
    @property
    def delta(self) -> float:
        """
        ## Tire Steered Angle

        Calculates tire steered angle in radians

        Parameters
        ----------
        None

        Returns
        -------
        float
            Tire steered angle in radians
        """
        return self.steered_angle + self.static_toe

    @property
    def gamma(self):
        """
        ## Tire Inclination Angle

        Calculates tire inclination angle in radians

        Parameters
        ----------
        None

        Returns
        -------
        float
            Tire inclination angle in radians
        """
        pass
    
    @property
    def center(self):
        """
        ## Tire Center

        Calculates the centroid of the tire

        Parameters
        ----------
        None

        Returns
        -------
        Sequence[float]
            Centroid of the tire
        """
        return self.center_node.position

    @property
    def direction(self):
        """
        ## Tire Direction

        Calculates unit vector acting through primary axis of tire (treated as a cylinder geometrically)

        Parameters
        ----------
        None

        Returns
        -------
        float
            Tire direction unit vector
        """
        pt_1 = np.array(self.contact_patch.position)
        pt_2 = np.array(self.center_node.position)
        pt_3 = np.array(self.front_node.position)
        
        vec_12 = pt_2 - pt_1
        vec_13 = pt_3 - pt_1

        return np.cross(vec_12, vec_13)

    def __str__(self):
        tire_name = self.tire.tire_name
        tire_center = self.center_node.position

        return f"Tire Name: {tire_name}\nTire Center: {tire_center}"