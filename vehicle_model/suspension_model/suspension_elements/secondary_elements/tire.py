from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node

from LHR_tire_toolkit.MF52 import MF52 # type: ignore

class Tire:
    def __init__(self, tire: MF52, contact_patch: Node, outer_diameter: float, width: float, inner_diameter: float) -> None:
        self.tire = tire
        self.contact_patch = contact_patch
        self.outer_diameter = outer_diameter
        self.width = width
        self.inner_diameter = inner_diameter
        self.steered_angle: float