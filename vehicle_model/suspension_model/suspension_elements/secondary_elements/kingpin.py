from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
import numpy as np


class Kingpin(Link):
    """
    ## Kingpin

    Kingpin object
    - Similar to beam, defined by two nodes

    Parameters
    ----------
    lower_node : Node
        Node representing lower end of the kingpin
    upper_node : Node
        Node representing upper end of the kingpin
    """
    def __init__(self, lower_node: Node, upper_node: Node) -> None:
        super().__init__(inboard=lower_node, outboard=upper_node)

        self.initial_length = self.length

    @property
    def length(self) -> float:
        """
        ## Length

        Length of physical kingpin

        Returns
        -------
        float
            Length of physical kingpin
        """
        return np.linalg.norm(self.inboard_node.position - self.outboard_node.position)