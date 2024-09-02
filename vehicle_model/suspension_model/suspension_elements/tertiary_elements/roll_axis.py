from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node


class RollAxis(Link):
    """
    ## Roll Axis

    DEPRECATED
    """
    def __init__(self, Fr_RC: Node, Rr_RC: Node) -> None:
        super().__init__(inboard=Fr_RC, outboard=Rr_RC)