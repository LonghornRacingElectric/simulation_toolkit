from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from typing import Union
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

    free_length : Union[float, int]
        Free length of spring

    rate : Union[float, int]
        Spring rate (force per unit compression)
    """
    def __init__(self, inboard_node: Node, outboard_node: Node, free_length: Union[float, int], rate: Union[float, int]) -> None:
        super().__init__(inboard_node=inboard_node, outboard_node=outboard_node, compliance=rate)

        self.compliance: Union[float, int]
        self.free_length = free_length

    @property
    def compression(self) -> Union[float, int]:
        """
        ## Compression

        Spring compression

        Returns
        -------
        Union[float, int]
            Spring compression
        """
        comp = self.free_length - self.length

        if comp < 0:
            warnings.warn("Requested coil spring is in tension")

        return comp

    @property
    def force(self) -> Union[float, int]:
        """
        ## Force

        Spring force

        Returns
        -------
        Union[float, int]
            Spring force
        """
        return self.compliance * self.compression