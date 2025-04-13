from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from LHR_tire_toolkit.MF52 import MF52 # type: ignore

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
        self.center = Node(position=[contact_patch[0], contact_patch[1], outer_diameter / 2])
        self.contact_patch = contact_patch
        self.outer_diameter = outer_diameter
        self.width = width
        self.inner_diameter = inner_diameter
        self.steered_angle: float = 0.0

        self.static_toe: float = static_toe
        self.static_gamma: float = static_gamma

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
        pass

    def __str__(self):
        tire_name = self.tire.tire_name
        tire_center = self.center.position

        return f"Tire Name: {tire_name}\nTire Center: {tire_center}"