from src.vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

import warnings


class Spring(Link):
    """
    ## Spring

    Spring object
    - Represents coil springs specifically

    Parameters
    ----------
    inboard : Node
        Node representing inboard end of linkage

    outboard : Node
        Node representing outboard end of linkage

    free_length : float
        Free length of spring

    rate : float
        Spring rate (force per unit compression)
    """
    def __init__(self, inboard_node: Node, outboard_node: Node, free_length: float, rate: float) -> None:
        super().__init__(inboard_node=inboard_node, outboard_node=outboard_node, compliance=rate)

        self.compliance: float
        self.free_length = free_length

    @property
    def compression(self) -> float:
        """
        ## Compression

        Spring compression

        Returns
        -------
        float
            Spring compression
        """
        comp = self.free_length - self.length

        if comp < 0:
            warnings.warn("Requested coil spring is in tension")

        return comp

    @property
    def force(self) -> float:
        """
        ## Force

        Spring force

        Returns
        -------
        float
            Spring force
        """
        return self.compliance * self.compression