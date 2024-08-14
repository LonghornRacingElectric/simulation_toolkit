from suspension_model.assets.misc_linalg import rotation_matrix
from suspension_model.suspension_elements.link import Link
from suspension_model.suspension_elements.node import Node
import numpy as np


class Kingpin(Link):
    def __init__(self, lower_node: Node, upper_node: Node) -> None:
        super().__init__(inboard=lower_node, outboard=upper_node)

        self.initial_length = self.length

    @property
    def length(self):
        return np.linalg.norm(self.inboard_node.position - self.outboard_node.position)