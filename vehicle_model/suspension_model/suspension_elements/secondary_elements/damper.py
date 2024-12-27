from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from typing import Sequence, Tuple, Union
import numpy as np

class Damper(Link):
    """
    ## Damper

    Damper object
    - Represents shock

    Parameters
    ----------
    inboard : Node
        Node representing inboard end of damper

    outboard : Node
        Node representing outboard end of damper
    
    damping_curve : Sequence[Tuple[float, float]]
        Lookup table for damping values, in the form: Sequence[Tuple[`velocity`, `force`]]
    """
    def __init__(self, inboard_node: Node, outboard_node: Node, damping_curve: Sequence[Tuple[float, float]]) -> None:
        super().__init__(inboard_node=inboard_node, outboard_node=outboard_node)
        
        self.damping_curve = list(zip(*damping_curve))
        self.velocity_reference: Sequence[float] = self.damping_curve[0]
        self.force_reference: Sequence[float] = self.damping_curve[1]
        
        self.velocity: float = 0
    
    @property
    def force(self) -> float:
        """
        ## Force

        Damper force at current velocity

        Returns
        -------
        float
            Damper force
        """
        return np.interp(self.velocity, self.velocity_reference, self.force_reference).__float__()